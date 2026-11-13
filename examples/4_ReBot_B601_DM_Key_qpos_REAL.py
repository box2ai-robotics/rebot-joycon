
if __name__ == "__main__":
    from runtime_log import start_runtime_log
    start_runtime_log()

# code by Boxjod 2025.12.01 Box2AI-Robotics copyright 盒桥智能 版权所有

import os
from pathlib import Path
from rebot_port import select_port
from importlib import import_module
ReBotCalibration = import_module("0_calibrate_rebot").ReBotCalibration
import mujoco
import mujoco.viewer
import numpy as np
import time

from ReBot_Kinematics import rebot_IK, rebot_FK, get_robot, get_home_q
from pynput import keyboard
import threading

from ReBot_SDK import ReBot
from runtime_log import record_event

# Learning_Model 的默认遥操直接使用此控制器及回中参数。
shared_control = import_module("6_ReBot_B601_DM_Joycon_Gpos_REAL")

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
GRIPPER_INCREMENT = 0.02  # rad/10 ms，按实际经过时间缩放
CONTROL_PERIOD = 0.01  # 目标 100 Hz；串口与计算超时会降低实际频率
VIEWER_PERIOD = 1.0 / 30.0

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
home_requested = False

# Handle key press events
def on_press(key):
    try:
        k = key.char.lower()  # Convert to lowercase to handle both upper and lower case inputs
        record_event("key_press", key=k)
        if k in key_to_joint_increase:
            with lock:
                keys_pressed[k] = 1  # Increase direction
        elif k in key_to_joint_decrease:
            with lock:
                keys_pressed[k] = -1  # Decrease direction
        elif k == "0":
            with lock:
                global home_requested
                home_requested = True
        
    except AttributeError:
        pass  # Handle special keys if necessary

# Handle key release events
def on_release(key):
    try:
        k = key.char.lower()
        record_event("key_release", key=k)
        with lock:
            keys_pressed.pop(k, None)
    except AttributeError:
        pass  # Handle special keys if necessary

# Start the keyboard listener in a separate thread
listener = keyboard.Listener(on_press=on_press, on_release=on_release)

# Backup for target_gpos in case of invalid IK
target_qpos_last = init_qpos.copy()

arm_tty = select_port()

# init ReBot Real Hardware
rebot_right = None
# rebot_right.if_print = True
normal_exit = False
try:
    rebot_right = ReBot(driver_port=arm_tty, arm_type="rebot_b601_dm", side="right")
    rebot_right.Go_Home_Pose()
    listener.start()
    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        viewer.cam.azimuth = 45    # 水平角度
        viewer.cam.elevation = -30  # 垂直角度
        viewer.cam.distance = 1.0   # 观察距离
        viewer.cam.lookat = [-0.1, 0.2, 0.3]  # 观察点坐标
    
        start = time.monotonic()
        home_cycle = 0
        home_done, home_limited = True, False
        home_start = start
        last_step = start - CONTROL_PERIOD
        next_viewer = 0.0
        report_start = start
        report_cycles = 0
        while viewer.is_running() and time.monotonic() - start < 1000:
            step_start = time.monotonic()
            dt = np.clip(step_start - last_step, 0.0, 2 * CONTROL_PERIOD)
            last_step = step_start
            with lock:
                if home_requested:
                    home_cycle = 0
                    home_done, home_limited = False, True
                    home_start = step_start
                    target_qpos = init_qpos.copy()
                    home_requested = False
                home_cycle += 1
                homing = home_cycle <= shared_control.STARTUP_HOME_CYCLES or not home_done
                for k, direction in ([] if homing else keys_pressed.items()):
                    joint_idx = (key_to_joint_increase[k] if k in key_to_joint_increase
                                 else key_to_joint_decrease[k])
                    increment = GRIPPER_INCREMENT if joint_idx == 6 else JOINT_INCREMENT
                    target_qpos[joint_idx] += increment * direction * dt / CONTROL_PERIOD
            target_qpos = np.clip(target_qpos, control_qlimit[0], control_qlimit[1])

            if home_cycle < shared_control.STARTUP_HOME_CYCLES or not home_done:
                if step_start - home_start > 30.0:
                    raise TimeoutError("限速回中超时，停止控制并失能")
                home_done = rebot_right.Go_Home_Pose(max_velocity=0.35 if home_limited else None)
                target_qpos = np.asarray(rebot_right.target_pose, dtype=float).copy()
            elif not homing:
                rebot_right.ReBot_Control(target_qpos)
                target_qpos = np.asarray(rebot_right.target_pose, dtype=float).copy()

            # 发包先于预览；只显示目标，不让物理步进影响控制节奏。
            now = time.monotonic()
            if now >= next_viewer:
                with viewer.lock():
                    mjdata.qpos[qpos_indices] = target_qpos[:6]
                    finger_opening = np.clip((target_qpos[6] - gripper_close) / (gripper_open - gripper_close), 0.0, 1.0) * mjmodel.joint("finger_left").range[1]
                    mjdata.qpos[finger_indices] = [finger_opening, -finger_opening]
                    mjdata.ctrl[:] = np.r_[target_qpos[:6], finger_opening]
                    mujoco.mj_forward(mjmodel, mjdata)
                viewer.sync()
                next_viewer = now + VIEWER_PERIOD
            report_cycles += 1
            if now - report_start >= 1.0:
                if not homing:
                    print(f"控制 {report_cycles / (now - report_start):.1f} Hz | "
                          f"夹爪目标/反馈 {target_qpos[6]:.3f}/{rebot_right.motors_pose[6]:.3f} rad | "
                          f"电机位置闭环 | 反馈力矩 {rebot_right.motors_torque[6]:.3f} Nm")
                report_start, report_cycles = now, 0
            time_until_next_step = CONTROL_PERIOD - (time.monotonic() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        normal_exit = True

except KeyboardInterrupt:
    print("User interrupted the simulation.")
finally:
    listener.stop()
    if rebot_right is not None:
        try:
            if normal_exit:
                for _ in range(shared_control.EXIT_HOME_CYCLES):
                    rebot_right.Go_Home_Pose()
                target_qpos = np.asarray(rebot_right.target_pose, dtype=float).copy()
        finally:
            try:
                rebot_right.ReBot_Disable()
            finally:
                rebot_right.MotorController.serial_.close()
