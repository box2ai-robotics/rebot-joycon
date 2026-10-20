"""仿真位姿预览：没有电机力矩反馈，不执行 1 N·m 实机保护。
实机例程及 LeRobot 共用 gripper_protection.py；此处不把双指接触力当作电机力矩。
"""


if __name__ == "__main__":
    from runtime_log import start_runtime_log
    start_runtime_log()

# code by Boxjod 2025.12.01 Box2AI-Robotics copyright 盒桥智能 版权所有

import os
import json
from pathlib import Path
from importlib import import_module
ReBotCalibration = import_module("0_calibrate_rebot").ReBotCalibration
import mujoco
import mujoco.viewer
import numpy as np
import time
import copy

from ReBot_Kinematics import rebot_IK, rebot_FK, get_robot, get_home_q
from joyconrobotics import JoyconRobotics

np.set_printoptions(linewidth=200)

# Set the MuJoCo render backend
os.environ["MUJOCO_GL"] = "egl"

# Define joint names
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Absolute path of the XML model
xml_path = str(Path(__file__).resolve().parents[1] / "URDF/reBot_B601_DM/scene.xml")
mjmodel = mujoco.MjModel.from_xml_path(xml_path)
qpos_indices = np.array([mjmodel.jnt_qposadr[mjmodel.joint(name).id] for name in JOINT_NAMES])
mjdata = mujoco.MjData(mjmodel)
finger_indices = [mjmodel.joint(name).qposadr[0] for name in ("finger_left", "finger_right")]
# 第七路保持夹爪电机角度；仿真时按标定端点映射为双指位移。
calibration = ReBotCalibration()
gripper_open = calibration.gripper_open
gripper_close = calibration.gripper_close

# Define joint control increment (in radians)
JOINT_INCREMENT = 0.01  # Can be adju`sted as needed
POSITION_INSERMENT = 0.0008
GRIPPER_TRAVEL_TIME_S = 1.0  # 与 6 号一致：标定全行程目标渐变 1 秒。

# create robot
robot = get_robot('rebot_b601_dm')

# Define joint limits
control_qlimit = calibration.control_limits(robot.qlim)

control_glimit = [[0.000, -0.450,  -0.00, -3.1, -3.1, -3.1], 
                  [0.450,  0.450,  0.680,  3.1,  3.1,  3.1]]

# Initialize target joint positions
init_qpos = np.r_[get_home_q(robot), gripper_close]
target_qpos = init_qpos.copy()  # Copy of initial joint positions
init_gpos = rebot_FK(init_qpos[:6], robot=robot)
# 窗口与 IK 种子从同一初始位姿开始；夹爪保持闭合。
mjdata.qpos[qpos_indices] = init_qpos[:6]
mjdata.qpos[finger_indices] = 0.0
mjdata.ctrl[:] = np.r_[init_qpos[:6], 0.0]
mujoco.mj_forward(mjmodel, mjdata)
# print(f'{init_gpos=}')
target_gpos = init_gpos.copy()

# Backup for target_gpos in case of invalid IK
target_gpos_last = init_gpos.copy()
target_qpos_last = init_qpos.copy()

# 重力补偿
def gravity_compensation(model, data):
    mujoco.mj_inverse(model, data)
    gravity_compensation_torques = data.qfrc_bias.copy()
    
    return gravity_compensation_torques

# Joycon
direction_data_r = [[], [], []]  
offset_position_m = init_gpos[0:3]
joyconrobotics_right = JoyconRobotics(device="right", 
                                      glimit = control_glimit,
                                      dof_speed = [2 / 3, 2 / 3, 2 / 3, 1, 1, 1],
                                      offset_position_m=offset_position_m, 
                                      pitch_down_double = True,
                                      gripper_close = gripper_close,
                                      gripper_open = gripper_open,
                                      gripper_state = gripper_close,
                                      )

t = 0
try:
    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        viewer.cam.azimuth = 45    # 水平角度
        viewer.cam.elevation = -30  # 垂直角度
        viewer.cam.distance = 1.0   # 观察距离
        viewer.cam.lookat = [-0.1, 0.2, 0.3]  # 观察点坐标
    
        gripper_tick = time.monotonic()
        start = time.time()
        while viewer.is_running() and time.time() - start < 1000:
            step_start = time.time()

            if t ==0 :
                mjdata.qpos[qpos_indices] = init_qpos[:6]
                finger_opening = np.clip((init_qpos[6] - gripper_close) / (gripper_open - gripper_close), 0.0, 1.0) * mjmodel.joint("finger_left").range[1]
                mjdata.qpos[finger_indices] = [finger_opening, -finger_opening]
                mjdata.ctrl[:] = np.r_[init_qpos[:6], finger_opening]
                mujoco.mj_step(mjmodel, mjdata)
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()
                
            t = t + 1
            now = time.monotonic()
            gripper_dt = min(max(now - gripper_tick, 0.0), 0.1)
            gripper_tick = now
            target_gpos, gripper_state_r, _ = joyconrobotics_right.get_control()# update()
            target_gpos = np.asarray(target_gpos, dtype=float).copy()
            if target_gpos.shape != (6,) or not np.isfinite(target_gpos).all() or not np.isfinite(gripper_state_r):
                raise ValueError("Non-finite or invalid Joy-Con target")
            gripper_state_r = np.clip(gripper_state_r, control_qlimit[0][6], control_qlimit[1][6])
            gripper_speed = abs(gripper_open - gripper_close) / GRIPPER_TRAVEL_TIME_S
            # 仿真以上次成功应用的目标为起点，对应 6 号的 last_sent。
            target_qpos[6] = target_qpos_last[6] + np.clip(
                gripper_state_r - target_qpos_last[6],
                -gripper_speed * gripper_dt, gripper_speed * gripper_dt)
            target_gpos[4] = -target_gpos[4]         
            target_gpos[3] = -target_gpos[3]   
                 
            print("target_gpos:", [f"{x:.3f}" for x in target_gpos])
            fd_qpos = np.clip(mjdata.qpos[qpos_indices], robot.qlim[0], robot.qlim[1])
            qpos_inv, ik_success = rebot_IK(fd_qpos, target_gpos, robot=robot)
            
            if ik_success:  # Check if the inverse kinematics solution is valid
                target_qpos[:6] = qpos_inv
                # Step the simulation
                for idx in range(7):
                    if target_qpos[idx] < control_qlimit[0][idx]:
                        target_qpos[idx] = control_qlimit[0][idx]
                    elif target_qpos[idx] > control_qlimit[1][idx]:
                        target_qpos[idx] = control_qlimit[1][idx]
                        
                for idx in range(2):
                    
                    mjdata.qpos[qpos_indices] = target_qpos[:6]
                    finger_opening = np.clip((target_qpos[6] - gripper_close) / (gripper_open - gripper_close), 0.0, 1.0) * mjmodel.joint("finger_left").range[1]
                    mjdata.qpos[finger_indices] = [finger_opening, -finger_opening]
                    mjdata.ctrl[:] = np.r_[target_qpos[:6], finger_opening]
                    # mjdata.ctrl[qpos_indices] = target_qpos
                    # gravity = gravity_compensation(model=mjmodel, data=mjdata)
                    # mjdata.qfrc_applied[qpos_indices] = gravity  # 重力补偿
                    mujoco.mj_step(mjmodel, mjdata)

                    # Update viewer options (e.g., toggle contact point display every second)
                    with viewer.lock():
                        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                    viewer.sync()
                
                target_qpos_last = target_qpos.copy()  # Backup the valid target_qpos
            else:
                target_qpos = target_qpos_last.copy()  # Revert to the last valid position if IK fails
            
            # Time management to maintain simulation timestep

            time_until_next_step = 0.001 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    viewer.close()
