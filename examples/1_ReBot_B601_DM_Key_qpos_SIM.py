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

from ReBot_Kinematics import rebot_IK, rebot_FK, get_robot, get_home_q, manual_gravity_compensation
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

# create robot
robot = get_robot('rebot_b601_dm')

# Define joint limits
control_qlimit = calibration.control_limits(robot.qlim)

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
target_qpos_last = init_qpos.copy()

# Thread-safe lock
lock = threading.Lock()

# Define key mappings for joint control
key_to_joint_increase = {
    '1': 0,  # joint1
    '2': 1,  # joint2
    '3': 2,  # joint3
    '4': 3,  # joint4
    '5': 4,  # joint5
    '6': 5,  # joint6
    '7': 6,  # gripper
}

key_to_joint_decrease = {
    'q': 0,  # joint1
    'w': 1,  # joint2
    'e': 2,  # joint3
    'r': 3,  # joint4
    't': 4,  # joint5
    'y': 5,  # joint6
    'u': 6,  # gripper
}

# Dictionary to track currently pressed keys and their direction
keys_pressed = {}

homing = False
HOME_MAX_VELOCITY = 0.35  # 七路按 0 回中目标速度 rad/s

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
            with lock:
                global homing
                homing = True

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

# Backup for target_gpos in case of invalid IK
target_qpos_last = init_qpos.copy()
try:
    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        viewer.cam.azimuth = 45    # 水平角度
        viewer.cam.elevation = -30  # 垂直角度
        viewer.cam.distance = 1.0   # 观察距离
        viewer.cam.lookat = [-0.1, 0.2, 0.3]  # 观察点坐标
        viewer._key_callback = None
    
        start = time.time()
        last_home_tick = time.monotonic()
        while viewer.is_running() and time.time() - start < 1000:
            step_start = time.time()

            now = time.monotonic()
            home_step = HOME_MAX_VELOCITY * np.clip(now - last_home_tick, 0.0, 0.02)
            last_home_tick = now
            # Update target_qpos based on the keys currently pressed
            with lock:
                returning_home = homing
                if homing:
                    target_qpos += np.clip(init_qpos - target_qpos, -home_step, home_step)
                    homing = not np.allclose(target_qpos, init_qpos, atol=1e-6, rtol=0)
                for k, direction in ([] if returning_home else keys_pressed.items()):
                    if k in key_to_joint_increase:
                        joint_idx = key_to_joint_increase[k]
                        if target_qpos[joint_idx] < control_qlimit[1][joint_idx]:
                            target_qpos[joint_idx] += (GRIPPER_INCREMENT if joint_idx == 6 else JOINT_INCREMENT) * direction
                        
                    elif k in key_to_joint_decrease:
                        joint_idx = key_to_joint_decrease[k]
                        if target_qpos[joint_idx] > control_qlimit[0][joint_idx]:
                            target_qpos[joint_idx] += (GRIPPER_INCREMENT if joint_idx == 6 else JOINT_INCREMENT) * direction  # direction is -1
            
            target_qpos = target_qpos

            target_qpos[6] = np.clip(target_qpos[6], control_qlimit[0][6], control_qlimit[1][6])

            for idx in range(2):
                mjdata.qpos[qpos_indices] = target_qpos[:6]
                finger_opening = np.clip((target_qpos[6] - gripper_close) / (gripper_open - gripper_close), 0.0, 1.0) * mjmodel.joint("finger_left").range[1]
                mjdata.qpos[finger_indices] = [finger_opening, -finger_opening]
                mjdata.ctrl[:] = np.r_[target_qpos[:6], finger_opening]
                mujoco.mj_step(mjmodel, mjdata)
                # Update viewer options (e.g., toggle contact point display every second)
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()
            
            target_qpos_last = target_qpos.copy()  # Backup the valid target_qpos
            
            print("target_gpos:", [f"{x:.3f}" for x in target_qpos])
            # Time management to maintain simulation timestep
            time_until_next_step = 0.001 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    listener.stop()  # Stop the keyboard listener
    viewer.close()
