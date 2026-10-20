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
from pynput import keyboard
import threading

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
JOINT_INCREMENT = 0.01  # Can be adjusted as needed
GRIPPER_INCREMENT = 0.02  # rad/循环，夹爪目标步长加快 2 倍
POSITION_INSERMENT = 0.0028 * (2 / 3)
CONTROL_PERIOD = 0.01  # 100 Hz；平移步长对应约 0.08 m/s

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

# Thread-safe lock
lock = threading.Lock()

# Define key mappings
key_to_joint_increase = {
    'w': 0,  # Move forward
    'a': 1,  # Move right
    'r': 2,  # Move up
    'e': 3,  # Roll +
    'g': 4,  # Pitch +
    'z': 5,  # yaw +
    '1': 6,  # Gripper +
}

key_to_joint_decrease = {
    's': 0,  # Move backward
    'd': 1,  # Move left
    'f': 2,  # Move down
    'q': 3,  # Roll -
    't': 4,  # Pitch -
    'c': 5,  # Gripper -
    '3': 6,  # Gripper -
}

# Dictionary to track currently pressed keys and their direction
keys_pressed = {}

# Handle key press events
def on_press(key):
    try:
        k = key.char.lower()  # Convert to lowercase to handle both upper and lower case inputs
        if k in key_to_joint_increase:
            with lock:
                keys_pressed[k] = 1  # Increase direction
        elif k in key_to_joint_decrease:
            with lock:
                keys_pressed[k] = -1  # Decrease direction
        elif k == "0":
            # Reset to initial joint positions
            print(f'{k=}')
            with lock:
                global target_qpos, target_gpos, target_gpos_last
                target_qpos = init_qpos.copy()
                target_gpos = init_gpos.copy()
                target_gpos_last = init_gpos.copy()
        
    except AttributeError:
        pass  # Handle special keys if necessary

# Handle key release events
def on_release(key):
    try:
        k = key.char.lower()
        if k in keys_pressed:
            with lock:
                del keys_pressed[k]
    except AttributeError:
        pass  # Handle special keys if necessary

# Start the keyboard listener in a separate thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 重力补偿
def gravity_compensation(model, data):
    mujoco.mj_inverse(model, data)
    gravity_compensation_torques = data.qfrc_bias.copy()
    
    return gravity_compensation_torques

# Backup for target_gpos in case of invalid IK
target_gpos_last = init_gpos.copy()
target_qpos_last = init_qpos.copy()
try:
    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        viewer.cam.azimuth = 45    # 水平角度
        viewer.cam.elevation = -30  # 垂直角度
        viewer.cam.distance = 1.0   # 观察距离
        viewer.cam.lookat = [-0.1, 0.2, 0.3]  # 观察点坐标
    
        next_print = 0.0
        start = time.monotonic()
        while viewer.is_running() and time.monotonic() - start < 1000:
            step_start = time.monotonic()

            # Update target_qpos based on the keys currently pressed
            with lock:
                for k, direction in keys_pressed.items():
                    if k in key_to_joint_increase:
                        position_idx = key_to_joint_increase[k]
                        if position_idx == 6:  # for rad
                            if (target_qpos[position_idx]) < control_qlimit[1][position_idx] - GRIPPER_INCREMENT * direction:
                                target_qpos[position_idx] += GRIPPER_INCREMENT * direction
                        elif position_idx in [3,4,5]:  # for rad
                            if (target_gpos[position_idx]) < control_glimit[1][position_idx] - JOINT_INCREMENT * direction:
                                target_gpos[position_idx] += JOINT_INCREMENT * direction
                        else:
                            if target_gpos[position_idx] <= control_glimit[1][position_idx]:
                                # if position_idx in [4,5]:
                                #     direction = -direction
                                target_gpos[position_idx] += POSITION_INSERMENT * direction
                        
                    elif k in key_to_joint_decrease:
                        position_idx = key_to_joint_decrease[k]
                        if position_idx == 6: # for gripper
                            if (target_qpos[position_idx]) > control_qlimit[0][position_idx] - GRIPPER_INCREMENT * direction:
                                target_qpos[position_idx] += GRIPPER_INCREMENT * direction
                        elif position_idx in [3,4,5]: # for rad
                            if (target_gpos[position_idx]) > control_glimit[0][position_idx] - JOINT_INCREMENT * direction:
                                target_gpos[position_idx] += JOINT_INCREMENT * direction
                        else:
                            if target_gpos[position_idx] >= control_glimit[0][position_idx]:
                                # if position_idx in [4,5]:
                                #     direction = -direction
                                target_gpos[position_idx] += POSITION_INSERMENT * direction
                                
            # 只在末端目标改变时求解；以上次接受的六轴目标作为连续种子。
            if not np.array_equal(target_gpos, target_gpos_last):
                qpos_inv, ik_success = rebot_IK(
                    target_qpos[:6], target_gpos, robot=robot,
                    tol=1e-9, ilimit=30, slimit=1, quiet=True)
                if ik_success:
                    target_qpos[:6] = qpos_inv
                    target_gpos_last = target_gpos.copy()
                else:
                    target_gpos = target_gpos_last.copy()

            # 本例直接预览关节位姿；不让重力/位置伺服再次改动 IK 输出。
            # 即使 IK 失败或只操作夹爪，也必须刷新窗口。
            with viewer.lock():
                mjdata.qpos[qpos_indices] = target_qpos[:6]
                finger_opening = np.clip((target_qpos[6] - gripper_close) / (gripper_open - gripper_close), 0.0, 1.0) * mjmodel.joint("finger_left").range[1]
                mjdata.qpos[finger_indices] = [finger_opening, -finger_opening]
                mujoco.mj_forward(mjmodel, mjdata)
            viewer.sync()

            if step_start >= next_print:
                print("target_gpos:", [f"{x:.3f}" for x in target_gpos])
                next_print = step_start + 0.2

            time_until_next_step = CONTROL_PERIOD - (time.monotonic() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    listener.stop()  # Stop the keyboard listener
    viewer.close()
