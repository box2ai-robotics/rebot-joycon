# code by Boxjod 2025.12.01 Box2AI-Robotics copyright 盒桥智能 版权所有

from Motor_CAN import Motor, DM_Motor_Type, MotorControl, Control_Type
import serial
import json
from pathlib import Path
import copy
import time
import numpy as np
from ReBot_Kinematics import get_home_q, create_rebot_b601_dm, REBOT_JOINT_LIMITS
from importlib import import_module
from runtime_log import start_runtime_log, record_event
from gripper_protection import GripperTorqueProtection, GRIPPER_TORQUE_TRIP_NM
ReBotCalibration = import_module("0_calibrate_rebot").ReBotCalibration

# Motor-shaft command limit, shared by all real examples and LeRobot examples backend.
# Policy targets are shaped in joint space; ordinary example/Joy-Con calls
# keep their existing control loop unless smooth=True is explicitly selected.
POLICY_ARM_MAX_VELOCITY_RAD_S = 0.35
POLICY_ARM_MAX_ACCELERATION_RAD_S2 = 0.8
POLICY_ARM_RESPONSE_RAD_S = 4.0
POLICY_ARM_MAX_DT_S = 1.0 / 30.0
POLICY_ARM_RESTART_GAP_S = 0.2
# Motor-side MIT position loop, following BoxArm_SDK.controlMIT.
# These are motor gains, NOT a host torque controller or a 3 Nm torque cap.
GRIPPER_KP = 2.0
GRIPPER_KD = 1.0
GRIPPER_FEEDBACK_TIMEOUT_S = 0.2
GRIPPER_STARTUP_FEEDBACK_WAIT_S = 1.0

class ReBotArm:
  def __init__(self, driver_port="/dev/ttyACM0", arm_type="rebot_b601_dm", side = "right", if_connect_directly = True, calibration_path=None):
    if if_connect_directly:
      start_runtime_log()
    self._last_gripper_command_time = None
    self._gripper_protection = GripperTorqueProtection(record_event, driver_port)
    self._policy_arm_position = None
    self._policy_arm_velocity = np.zeros(6)
    self._policy_arm_time = None
    self.calibration_path = calibration_path
    self.driver_port = driver_port
    # 兼容旧示例的名称，实际配置统一为 reBot B601-DM。
    if arm_type not in ("rebot_b601_dm", "boxarm_b1"):
      raise ValueError(f"Unsupported arm_type: {arm_type}")
    self.arm_type = "rebot_b601_dm"
    self.side = side
    self.dt = 0.01
    self.conrol_diff_limit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2]  # 1*0.01 = 0.01 # rad/s
    self.conrol_diff_rate = [0.9, 0.9, 0.9, 0.5, 0.9, 0.9, 0.95]  # 1*0.01 = 0.01 # rad/s
    self.if_print = False
    
    self.init_rebot()
    
    self.is_connected_flag = False
    if if_connect_directly:
      self.Connetct_ReBot()
      try:
        # 电机可能保留上一个程序的 POS_VEL 模式；变绿不代表已处于 MIT。
        self.ReBot_Disable()
        self.Switch_Control_Mode("MIT")
        self.update_motors_status(wait_timeout_s=GRIPPER_STARTUP_FEEDBACK_WAIT_S)
        self.Enable_Arm()
      except Exception:
        try:
          self.ReBot_Disable()
        finally:
          self.MotorController.serial_.close()
          self.is_connected_flag = False
        raise

  def init_rebot(self):
    # 1.定义电机
    # MasterID和SlaveID需要在达妙上位机进行设置！！
    # 如果出现问题请先检查MasterID是否不和SlaveID冲突，并且不为0x00
    self.calibration = ReBotCalibration() if self.calibration_path is None else ReBotCalibration(self.calibration_path)
    self.motorid_list = self.calibration.ids.copy()
    self.masterid_list = [motor_id + 0x10 for motor_id in self.motorid_list]
    if set(self.masterid_list) & set(self.motorid_list):
      raise ValueError("电机 ID 与反馈 ID 冲突")
    self.motor_reverse = [1, 1, 1, 1, 1, 1, 1]

    if self.arm_type == "rebot_b601_dm":
      # 0 对应本控制库 DM4340，1 对应 DM4310；前三轴为 4340 系列。
      self.motor_type = [0, 0, 0, 1, 1, 1, 1]
    
      self.motors_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
      self.motors_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
      self.motors_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
      # 统一使用用户校准后的控制坐标，六轴取模型/校准范围交集。
      self.control_qlimit = self.calibration.control_limits(REBOT_JOINT_LIMITS.T).tolist()

      # 普通遥操 MIT Kp；回中单独降低 20%，缓和启动/按 0 时的纠正力度。
      # self.kp = [120.0, 120.0, 120.0, 18.0, 18.0, 18.0, 8.0]
      self.kp = [100.0, 100.0, 100.0, 24.0, 24.0, 24.0, GRIPPER_KP]
      self.home_kp_scale = 0.8
      
      # 官方前三轴 Kd=8；当前 Motor_CAN.controlMIT 的编码范围只有 0..5。
      # self.kd = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0, GRIPPER_KD]
      self.kd = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0, GRIPPER_KD]
      
      self.kp_control = copy.copy(self.kp)
      
      self.home_pose = np.r_[get_home_q(create_rebot_b601_dm(self.calibration)), self.calibration.gripper_close]
      self.zero_pose = self.home_pose.copy()  # 回零同样使用 TCP Z=0.25 m

      self.target_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]

      self.gravity_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]
      self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
      
    self.motors_len = len(self.motorid_list)
            
  def Connetct_ReBot(self):
    start_runtime_log()
    record_event("connect", port=self.driver_port, side=self.side,
                 calibration=str(self.calibration.path),
                 offsets=self.calibration.offsets.tolist(), signs=self.calibration.signs.tolist(),
                 motor_ids=list(self.motorid_list), home=self.home_pose.tolist(),
                 limits=self.control_qlimit, kp=list(self.kp), kd=list(self.kd),
                 gripper_control="motor_position", software_torque_cap_nm=None)
    print("夹爪：电机端 MIT 位置闭环；原 3 N·m 软件力矩限幅不再适用")
    print(f"使用校准文件：{self.calibration.path}")
    print("六轴软件零偏(rad):", np.round(self.calibration.offsets[:6], 6).tolist())
    
    self.Motors =[]
    for idx in range(self.motors_len):
        if self.motor_type[idx] == 0:
            self.Motors.append(Motor(DM_Motor_Type.DM4340, self.motorid_list[idx], self.masterid_list[idx]))
        elif self.motor_type[idx] == 1:
            self.Motors.append(Motor(DM_Motor_Type.DM4310, self.motorid_list[idx], self.masterid_list[idx]))
               
    serial_device = serial.Serial(self.driver_port, 921600, timeout=0.5)
    
    self.MotorController = MotorControl(serial_device)
    for idx in range(self.motors_len):
      self.MotorController.addMotor(self.Motors[idx])
    self.is_connected_flag = True
    
  def is_connected(self):
    return self.is_connected_flag
  
  def Enable_Arm(self):
    record_event("enable_begin", port=self.driver_port)
    # 2.使能全部电机
    for idx in range(self.motors_len):
      self.MotorController.enable(self.Motors[idx])
    record_event("enable_complete", port=self.driver_port)

  def Switch_Control_Mode(self,mode):
    record_event("mode_requested", port=self.driver_port, mode=mode)
    if mode != "MIT":
      raise ValueError("reBot SDK 当前只支持 MIT 模式")
    for motor in self.Motors:
      # 丢弃旧回包缓存，要求本次切换得到确认；不写入 Flash。
      motor.temp_param_dict.pop(10, None)
      if not self.MotorController.switchControlMode(motor, Control_Type.MIT):
        raise RuntimeError(f"电机 0x{motor.SlaveID:02X} 切换 MIT 模式尝试 10 次仍未获确认，停止使能")
    print("全部 7 路电机已确认 MIT 模式")
    record_event("mode_confirmed", port=self.driver_port, mode=mode)

  def Arm_Zero_Setting(self):
    # 3. 零位校准
    for idx in range(self.motors_len):
      self.MotorController.disable(self.Motors[idx])
      print(f"校准零位{idx}，按回车确认")
      input()
      self.MotorController.set_zero_position(self.Motors[idx])
      time.sleep(0.1)

  def update_motors_status(self, wait_timeout_s=GRIPPER_FEEDBACK_TIMEOUT_S):
    for motor in self.Motors:
      self.MotorController.refresh_motor_status(motor)
    # read_all() is non-blocking: the last (gripper) request can still be in
    # flight. After camera initialization or an inference pause, allow this
    # query a bounded reply window before declaring feedback lost. Fresh
    # feedback takes the fast path; no motion command is sent while waiting.
    deadline = time.monotonic() + wait_timeout_s
    next_request = time.monotonic() + 0.05
    while True:
      stamp = self.Motors[6].feedback_time
      now = time.monotonic()
      if stamp is not None and now - stamp <= GRIPPER_FEEDBACK_TIMEOUT_S:
        break
      if now >= deadline:
        record_event("feedback_timeout", port=self.driver_port, last_feedback_s=stamp,
                     wait_timeout_s=wait_timeout_s)
        self.ReBot_Disable()
        age = "从未收到" if stamp is None else f"距上次反馈 {now - stamp:.3f}s"
        raise RuntimeError(f"夹爪反馈缺失或超时（{age}，本次等待 {wait_timeout_s:.3f}s），已停止使能")
      time.sleep(min(0.005, deadline - now))
      self.MotorController.recv()
      if time.monotonic() >= next_request:
        self.MotorController.refresh_motor_status(self.Motors[6])
        next_request = time.monotonic() + 0.05

    # Copy after receiving: a late reply must also update the PD input.
    for idx in range(self.motors_len):
      direction = self.calibration.signs[idx]
      self.motors_pose[idx] = (self.Motors[idx].getPosition() - self.calibration.offsets[idx]) * direction
      self.motors_speed[idx] = self.Motors[idx].getVelocity() * direction
      self.motors_torque[idx] = self.Motors[idx].getTorque() * direction

    record_event("feedback", port=self.driver_port,
                 position=list(map(float, self.motors_pose)),
                 velocity=list(map(float, self.motors_speed)),
                 torque=list(map(float, self.motors_torque)),
                 feedback_time_s=[getattr(m, "feedback_time", None) for m in self.Motors])

    if self.if_print:
      print(f"self.target_pose_input={[float(x) for x in self.target_pose_input]}")
      print(f"self.target_pose={[float(x) for x in self.target_pose]}")
      print(f"self.motors_pose={[float(x) for x in self.motors_pose]}")
      # print(f"self.control_pose={[float(x) for x in self.control_pose]}")
      # print(f"self.motors_speed={[float(x) for x in self.motors_speed]}")
      # print(f"self.motors_torque={[float(x) for x in self.motors_torque]}")
      print()
  
  def ReBot_Safe_Check(self, diff_max = 1.5):
    # motor speed limit change the kp
  
    for idx in range(self.motors_len): 
      motor_position_diff = abs(self.target_pose[idx]) - abs(self.motors_pose[idx])
      if idx == 6 or motor_position_diff < self.conrol_diff_limit[idx] : # 0-0.7 = 0.7 > 0.01
        # self.control_pose[idx] = self.target_pose[idx]
        self.kp_control[idx] = self.kp[idx]
      else:
        # print(f'关节id{idx=}超速')
        self.kp_control[idx] = self.kp[idx] - (motor_position_diff / diff_max * self.kp[idx] * self.conrol_diff_rate[idx])
      # Large tracking errors must not produce negative gains that wrap in MIT encoding.
      self.kp_control[idx] = float(np.clip(self.kp_control[idx], 0.0, self.kp[idx]))

      # motor range limit and control
      if self.target_pose[idx] < self.control_qlimit[0][idx]:
          self.target_pose[idx] = self.control_qlimit[0][idx]
      elif self.target_pose[idx] > self.control_qlimit[1][idx]:
          self.target_pose[idx] = self.control_qlimit[1][idx]
          
  def _smooth_policy_arm_target(self):
    now = time.monotonic()
    measured = np.asarray(self.motors_pose[:6], dtype=float)
    goal = np.asarray(self.target_pose[:6], dtype=float)
    if not np.isfinite(measured).all() or not np.isfinite(goal).all():
      self.ReBot_Disable()
      raise ValueError("策略关节目标和反馈必须为有限数值")
    elapsed = None if self._policy_arm_time is None else now - self._policy_arm_time
    self._policy_arm_time = now
    limits = np.asarray(self.control_qlimit)[:, :6]
    goal = np.clip(goal, *limits)
    if elapsed is None or elapsed > POLICY_ARM_RESTART_GAP_S or elapsed < 0:
      # Start/restart at feedback, never jump to a model's first frame or
      # spend accumulated wall-clock time catching up after slow inference.
      self._policy_arm_position = np.clip(measured, *limits)
      self._policy_arm_velocity = np.zeros(6)
    else:
      dt = min(elapsed, POLICY_ARM_MAX_DT_S)
      omega = POLICY_ARM_RESPONSE_RAD_S
      acceleration = omega ** 2 * (goal - self._policy_arm_position) - 2 * omega * self._policy_arm_velocity
      acceleration = np.clip(acceleration, -POLICY_ARM_MAX_ACCELERATION_RAD_S2, POLICY_ARM_MAX_ACCELERATION_RAD_S2)
      velocity = np.clip(self._policy_arm_velocity + acceleration * dt,
                         -POLICY_ARM_MAX_VELOCITY_RAD_S, POLICY_ARM_MAX_VELOCITY_RAD_S)
      position = self._policy_arm_position + velocity * dt
      clipped = np.clip(position, *limits)
      self._policy_arm_velocity = np.where(position == clipped, velocity, 0.0)
      self._policy_arm_position = clipped
    self.target_pose[:6] = self._policy_arm_position.tolist()

  def ReBot_Control(self, target_pose, kp = 0, *, smooth=False):
    record_event("control_request", port=self.driver_port,
                 target=list(map(float, target_pose)), smooth=smooth)
    self.target_pose_input = target_pose
    for idx in range(7):
      self.target_pose[idx] = target_pose[idx]
    self.update_motors_status()
    if smooth:
      self._smooth_policy_arm_target()
    else:
      self._policy_arm_time = None
    self.ReBot_Safe_Check()  # 显式增益也保留关节位置裁剪。
    if not (np.isscalar(kp) and kp == 0):
      gains = np.full(self.motors_len, kp) if np.isscalar(kp) else np.asarray(kp, dtype=float)
      if gains.shape != (self.motors_len,) or not np.all(np.isfinite(gains)) or np.any(gains < 0) or np.any(gains > 500):
        raise ValueError("MIT Kp 必须为 0..500 范围内的标量或七路数组")
      self.kp_control = gains.tolist()
        
    for idx in range(self.motors_len):
      self._send_joint_command(idx)
    # time.sleep(self.dt)

  def ReBot_Gravity_Move(self, gravity):
    self.update_motors_status()
    # self.conrol_diff_limit = 0.2
    self.ReBot_Safe_Check(diff_max=1.5)
    
    for idx in range(self.motors_len):
      self.gravity_offset[idx] = gravity[idx] * self.motor_reverse[idx]
      # if idx == 1:
      self.gravity_offset[idx] = self.gravity_offset[idx] * 4
      if self.motors_torque[idx] < 0 and self.gravity_offset[idx] < 0 and self.motors_torque[idx] < self.gravity_offset[idx] :
        self.target_pose[idx] = self.motors_pose[idx]
      elif self.motors_torque[idx] > 0 and self.gravity_offset[idx] > 0 and self.motors_torque[idx] > self.gravity_offset[idx]:
        self.target_pose[idx] = self.motors_pose[idx]
      elif self.motors_torque[idx] * self.gravity_offset[idx] < 0 and abs(self.motors_torque[idx])>1:
        self.target_pose[idx] = self.motors_pose[idx]
      
      
      # elif gravity[idx] < 0 and -self.motors_torque[idx] < gravity[idx] * 2:
      #   self.target_pose[idx] = self.motors_pose[idx]
      # else:
      #   self.target_pose[idx] = self.motors_pose[idx]
      # self.gravity_offset[idx] = gravity[idx] * self.motor_reverse[idx]
      
      # if idx == 3:
      #   self.gravity_offset[idx] = -self.gravity_offset[idx]
        
      # motor range limit and control
      # if self.target_pose[idx] < self.control_qlimit[0][idx]:
      #   self.self.gravity_offset[idx] = 0
      # elif self.target_pose[idx] > self.control_qlimit[1][idx]:
      #   self.self.gravity_offset[idx] = 0
      
      self._send_joint_command(idx)
      
      # self.MotorController.controlMIT(DM_Motor=self.Motors[idx],
      #                             kp=0, kd=0,
      #                             q=0, dq=0, tau=self.gravity_offset[idx],)
    print("motors_torque:", [f"{x:.3f}" for x in self.motors_torque])
    print("gravity_offset:", [f"{x:.3f}" for x in self.gravity_offset ])
    print()
    
  def _protect_gripper_target(self):
    self.target_pose[6] = self._gripper_protection.apply(
        float(self.target_pose[6]), float(self.motors_pose[6]), float(self.motors_torque[6]))

  def _send_joint_command(self, idx):
    kp, kd, torque = self.kp_control[idx], self.kd[idx], 0.0
    control = "motor_position"
    if idx == 6:
      values = [self.target_pose[6], self.motors_pose[6], self.motors_speed[6],
                self.motors_torque[6], self.kp_control[6], self.kd[6]]
      if not np.isfinite(values).all():
        self.ReBot_Disable()
        raise ValueError("夹爪状态/增益必须为有限数值")
      self._protect_gripper_target()
      now = time.monotonic()
      elapsed = (None if self._last_gripper_command_time is None
                 else now - self._last_gripper_command_time)
      self._last_gripper_command_time = now
      record_event("gripper_command_attempt", port=self.driver_port,
                   target=float(self.target_pose[6]), position=float(self.motors_pose[6]),
                   velocity=float(self.motors_speed[6]),
                   feedback_torque=float(self.motors_torque[6]), elapsed_s=elapsed,
                   kp=float(kp), kd=float(kd),
                   feedforward_torque=torque, control=control)
    motor_position = float(self.target_pose[idx] * self.calibration.signs[idx]
                           + self.calibration.offsets[idx])
    record_event("joint_command_attempt", port=self.driver_port, joint=idx + 1,
                 kp=float(kp), kd=float(kd),
                 motor_position=motor_position, velocity=0.0, feedforward_torque=torque)
    # All seven motors use motor-side position PD; tau is zero feedforward.
    self.MotorController.controlMIT(DM_Motor=self.Motors[idx],
        kp=kp, kd=kd, q=motor_position, dq=0, tau=torque)
    record_event("joint_command_returned", port=self.driver_port, joint=idx + 1)

  def Go_Home_Pose(self):
    record_event("home", port=self.driver_port)
    self.ReBot_Control(self.home_pose, kp=np.asarray(self.kp) * self.home_kp_scale)
    
  def Go_Zero_Pose(self):
    record_event("zero", port=self.driver_port)
    self.ReBot_Control(self.zero_pose, kp=np.asarray(self.kp) * self.home_kp_scale)
     
  def ReBot_Disable(self):
    record_event("disable_begin", port=self.driver_port)
    self._policy_arm_time = None
    self._policy_arm_position = None
    self._policy_arm_velocity = np.zeros(6)
    self._last_gripper_command_time = None
    for idx in range(self.motors_len):
      self.MotorController.disable(self.Motors[idx])
    record_event("disable_complete", port=self.driver_port)


# 简短类名，与 ReBotArm 使用相同实现。
ReBot = ReBotArm
