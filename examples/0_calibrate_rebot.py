"""在规定机械零位运行：采集 B601-DM 六轴软件零偏及夹爪闭合端点，不写电机硬件零点。"""

if __name__ == "__main__":
    from runtime_log import start_runtime_log
    start_runtime_log()


import argparse
import json
import math
import time
from pathlib import Path

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
CALIBRATION_PATH = Path(__file__).resolve().parent / 'calibration/rebot.json'
# 与 Motor_CAN.MotorControl.Limit_Param 中 DM4310 的位置编码范围一致。
GRIPPER_ENCODER_LIMIT_RAD = 12.5


class ReBotCalibration:
    def __init__(self, path=CALIBRATION_PATH):
        self.path = Path(path).resolve()
        config = json.loads(self.path.read_text())
        snapshot = json.loads(self.path.with_suffix('.encoders.json').read_text())
        self.ids = []
        self.signs = []
        self.offsets = []
        limits = []
        for i, name in enumerate(JOINT_NAMES):
            row, sample = config[name], snapshot[name]
            motor_id = row['id']
            if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7EF or sample['id'] != motor_id:
                raise ValueError(f'{name}: 校准 ID 与零位校准不一致或超出范围')
            if row['drive_mode'] not in (0, 1):
                raise ValueError(f'{name}: drive_mode 必须为 0/1')
            if row['homing_offset'] != 0:
                raise ValueError(f'{name}: 六轴软件零偏取自零位校准，不支持叠加非零 homing_offset')
            self.ids.append(motor_id)
            self.signs.append(1 if row['drive_mode'] == 0 else -1)
            self.offsets.append(float(sample['position_rad']) if i < 6 else 0.0)
            limits.append([float(row['range_min']), float(row['range_max'])])
        if len(set(self.ids)) != 7:
            raise ValueError('校准电机 ID 重复')
        self.signs = np.asarray(self.signs, dtype=float)
        self.offsets = np.asarray(self.offsets)
        raw_limits = np.deg2rad(np.asarray(limits)).T
        if not np.isfinite(raw_limits).all() or not np.isfinite(self.offsets).all() or np.any(raw_limits[0] >= raw_limits[1]):
            raise ValueError('校准范围/零偏无效')
        self.limits = np.sort(self.from_motor(raw_limits), axis=0)
        # 夹爪仍使用电机角；实测闭合端点优先于配置中的名义 range_max=0。
        # 原始零点可落在闭合位置之前，不能把正的闭合读数裁成 0。
        open_angle = raw_limits[0, 6]
        close_angle = float(snapshot['gripper']['position_rad'])
        endpoints = np.array([open_angle, close_angle])
        if (not np.isfinite(endpoints).all()
                or np.any(np.abs(endpoints) > GRIPPER_ENCODER_LIMIT_RAD)):
            raise ValueError('夹爪端点无效')
        if close_angle - open_angle < 1e-6:
            raise ValueError('校准范围内没有有效夹爪行程')
        endpoints *= self.signs[6]
        self.gripper_open, self.gripper_close = endpoints
        self.limits[:, 6] = sorted(endpoints)

    def from_motor(self, raw):
        """模型/控制角 = direction * (电机原始角 - 零位零位校准)。"""
        return (np.asarray(raw) - self.offsets) * self.signs

    def to_motor(self, joint):
        return np.asarray(joint) * self.signs + self.offsets

    def control_limits(self, model_limits):
        """六轴同时遵守模型和校准范围；夹爪采用有效端点。"""
        limits = self.limits.copy()
        limits[0, :6] = np.maximum(limits[0, :6], model_limits[0])
        limits[1, :6] = np.minimum(limits[1, :6], model_limits[1])
        if np.any(limits[0] >= limits[1]):
            raise ValueError('校准范围与模型限位没有有效交集')
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
    raise TimeoutError(f"未收到有效反馈：{', '.join(missing)}；未保存文件")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="SDK 串口")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "calibration/rebot.json",
                        help="电机配置路径；软件零位校准保存到同名 .encoders.json")
    args = parser.parse_args()

    import serial
    from Motor_CAN import Motor, MotorControl, DM_Motor_Type

    # 直接采集未扣零偏的反馈：首次标定不依赖旧文件，重新标定不叠加旧零偏。
    motor_ids = list(range(1, 8))
    motors = [Motor(DM_Motor_Type.DM4340 if i < 3 else DM_Motor_Type.DM4310,
                    motor_id, motor_id + 0x10) for i, motor_id in enumerate(motor_ids)]
    print("按规定机械零位及夹爪闭合位采集；本次读数将作为后续控制的零位校准。")
    device = serial.Serial(args.port, 921600, timeout=0.5)
    try:
        controller = MotorControl(device)
        for motor in motors:
            controller.addMotor(motor)
        positions = read_positions(controller, motors)
    finally:
        device.close()

    calibration = {}
    encoders = {}
    for name, motor_id, q, (low, high) in zip(JOINTS, motor_ids, positions, LIMITS_DEG):
        calibration[name] = dict(id=motor_id, drive_mode=0, homing_offset=0,
                                 range_min=low, range_max=high)
        encoders[name] = dict(id=motor_id, position_rad=q, position_deg=math.degrees(q))
        print(f"{name:14s}: {q: .6f} rad / {math.degrees(q): .3f} deg")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    zero_reference = args.output.with_suffix(".encoders.json")
    zero_reference.write_text(json.dumps(encoders, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    args.output.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print(f"六轴零位/夹爪闭合校准：{zero_reference}\n电机 ID、方向与范围：{args.output}")
    print("1–6 号示例 SDK 将自动使用这两份默认路径文件；前六轴扣除软件零偏，夹爪保留电机角。")
    print("外部 LeRobot 驱动仍忽略 homing_offset，不会自动应用本 SDK 的 .encoders.json 软件零偏。")


if __name__ == "__main__":
    main()
