"""在规定机械零位运行：将 B601-DM 七路当前位置设为硬件零点并保存到 Flash。"""

if __name__ == "__main__":
    from runtime_log import start_runtime_log
    start_runtime_log()


import argparse
import math
import time

import numpy as np


# 对应本项目 LeRobot rebot_b601_follower/motor_family.py 的 DM_PROFILE。
JOINTS = (
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
    "wrist_yaw", "wrist_roll", "gripper",
)
LIMITS_DEG = ((-150, 150), (-200, 1), (-200, 1), (-80, 90),
              (-90, 90), (-90, 90), (-270, 0))


JOINT_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
               'wrist_yaw', 'wrist_roll', 'gripper')
class ReBotCalibration:
    """硬件零位坐标下的固定 ID、方向和行程；不读写标定文件。"""
    def __init__(self, path=None):
        if path is not None:
            raise ValueError("已改用电机 Flash 零位，请移除 calibration_path/software_calibration 参数")
        self.ids = list(range(1, 8))
        self.signs = np.ones(7)
        self.offsets = np.zeros(7)
        self.limits = np.deg2rad(np.asarray(LIMITS_DEG, dtype=float)).T
        self.gripper_open = self.limits[0, 6]
        self.gripper_close = 0.0

    def from_motor(self, raw):
        return np.asarray(raw) * self.signs

    def to_motor(self, joint):
        return np.asarray(joint) * self.signs

    def control_limits(self, model_limits):
        """六轴取固定行程与模型限位交集；夹爪闭合位置为硬件零点。"""
        limits = self.limits.copy()
        limits[0, :6] = np.maximum(limits[0, :6], model_limits[0])
        limits[1, :6] = np.minimum(limits[1, :6], model_limits[1])
        if np.any(limits[0] >= limits[1]):
            raise ValueError('固定行程与模型限位没有有效交集')
        return limits


def read_positions(controller, motors):
    """等待七路实际反馈，避免把 SDK 初始的 0 当作编码器读数。"""
    for motor in motors:
        motor.state_q = math.nan
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        for motor in motors:
            controller.refresh_motor_status(motor)
        positions = [float(motor.getPosition()) for motor in motors]
        if all(math.isfinite(q) for q in positions):
            return positions
        time.sleep(0.02)
    missing = [name for name, q in zip(JOINTS, positions) if not math.isfinite(q)]
    raise TimeoutError(f"未收到有效反馈：{', '.join(missing)}；停止校准")


def check_hardware_zero(positions):
    bad = [name for name, q in zip(JOINTS, positions)
           if not math.isfinite(q) or abs(q) > 0.02]
    if bad:
        raise RuntimeError(f"硬件零位核验失败（容差 0.02 rad）：{', '.join(bad)}；"
                           "不要启动控制，请检查机械姿态、置零及断电保存是否成功")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="SDK 串口")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--hardware-zero", action="store_true", default=True,
                      help="默认行为：将七路当前位置置零并保存到电机 Flash，可省略此参数")
    mode.add_argument("--verify-hardware-zero", action="store_true",
                      help="断电重启后，在同一机械零位只读电机核验，不写电机或本地文件")
    args = parser.parse_args()

    import serial
    from Motor_CAN import Motor, MotorControl, DM_Motor_Type

    # 直接采集未扣零偏的反馈：首次标定不依赖旧文件，重新标定不叠加旧零偏。
    motor_ids = list(range(1, 8))
    motors = [Motor(DM_Motor_Type.DM4340 if i < 3 else DM_Motor_Type.DM4310,
                    motor_id, motor_id + 0x10) for i, motor_id in enumerate(motor_ids)]
    print("请保持规定机械零位及夹爪闭合位并支撑机械臂；默认写入电机零点和 Flash，核验模式只读。")
    device = serial.Serial(args.port, 921600, timeout=0.5)
    try:
        controller = MotorControl(device)
        for motor in motors:
            controller.addMotor(motor)
        positions = read_positions(controller, motors)
        if args.verify_hardware_zero:
            check_hardware_zero(positions)
            print("七路硬件零位核验通过：", positions)
            return
        if args.hardware_zero:
            print("置零前七路位置(rad)：", positions)
            print("写入七路硬件零点；请保持机械零位及夹爪闭合位，支撑机械臂。")
            for motor in motors:
                controller.disable(motor)
            for motor in motors:
                controller.set_zero_position(motor)
            positions = read_positions(controller, motors)
            check_hardware_zero(positions)
            for motor in motors:
                controller.save_motor_param(motor)
            positions = read_positions(controller, motors)
            check_hardware_zero(positions)
            print("已收到七路 Flash 保存应答并核验当前零位；建议断电重启后只读核验。")
    finally:
        device.close()

    print("七路位置(rad)：", positions)
    print("零位保存在电机 Flash；1–6 号示例使用硬件零位坐标，不再读取本地标定文件。")


if __name__ == "__main__":
    main()
