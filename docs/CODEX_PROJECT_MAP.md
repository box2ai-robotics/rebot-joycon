# 任务检索地图

路径相对当前项目根；只列已核实入口。examples 开发测试已在发布整理时移出仓库，相关离线回归结果见 [RUNBOOK](RUNBOOK.md)，不再作为可直接运行的测试入口。

| 任务 | 权威文件 / 符号 | 直接验证 |
| --- | --- | --- |
| 直接运行示例 / 已有策略 | [零位校准](../README_zh.md#零位校准与使用)；[运行示例](../README_zh.md#运行)：1–6 号独立命令及各控制方式按键；[运行策略](../Learning_Model/README_zh.md#运行策略)；`Learning_Model/run_rebot_policy.py`：`main`、`POLICY_PATH` | README 命令 Bash 语法、入口路径、按键映射和新增本地链接静态检查；推理需已有完整模型，脚本检查缺失文件后停止 |
| 遥操启动缺少 `teleop` / `robot` 配置 | [真机遥操](../Learning_Model/README_zh.md#真机遥操)的完整命令；`lerobot_teleoperate.TeleoperateConfig` | Bash 语法检查与仅配置解析；未定义 `REBOT_ARGS` 时展开为空，复现缺字段错误；不启动设备 |
| LeRobot 离线自检回中报 `Invalid gripper command or torque feedback` | `Learning_Model/lerobot/tests/teleoperators/test_joycon.py`：`test_calibrated_robot_home_gains_and_timeout`；follower `_send_gripper` | 模拟反馈须含 `pos/vel/torq`；六轴检查电机端增益，MotorBridge 夹爪检查主机 PD 限幅力矩；三文件自检命令与 136 项通过记录见 [RUNBOOK](RUNBOOK.md) |
| LeRobot 相机 MJPG 采集格式 | `Learning_Model/run_rebot_policy.py`：`cameras`；`Learning_Model/README.md`：相机命令参数；LeRobot `OpenCVCamera._validate_fourcc` | 入口配置及文档 JSON 静态核对；保持 640×480、30 FPS；相机实测待验证 |
| Flash 零位与无文件串口选择 | `examples/rebot_port.py`；SDK `Connetct_ReBot`；0 号；`Motor_CAN.__process_set_param_packet` | 历史模拟串口回归；无本地记录可运行，最新结果见 RUNBOOK |
| 硬件零点及断电保存 | `examples/0_calibrate_rebot.py`：默认硬件置零（`--hardware-zero` 可省略）、`--verify-hardware-zero`；`Motor_CAN.save_motor_param` | 历史模拟串口回归；[协议依据与边界](research/hardware_zero.md)；断电实测待用户核验 |
| 第 7 路 4310/4340 自动适配 | `examples/ReBot_SDK.py`：`Connetct_ReBot`；`Motor_CAN.py`：`read_motor_param` | 历史离线回归：四种量程、正负力矩、缺失/异常参数、延迟回复与旧缓存；仅 SDK 后端 |
| 全部实机夹爪反馈超过 1 N·m 收回目标 | `examples/gripper_protection.py`：`GripperTorqueProtection.apply`、`GRIPPER_TORQUE_TRIP_NM`；SDK `_protect_gripper_target`；LeRobot follower `_send_gripper` | 历史离线回归：正负超限、锁存、反向卸力、键盘无目标积累；LeRobot follower/example6 测试：多模式、双臂隔离、修正动作返回与插补 |
| 仿真 Joy-Con 夹爪开合 | `examples/3_ReBot_B601_DM_Joycon_Gpos_SIM.py`：`GRIPPER_TRAVEL_TIME_S`、`gripper_dt`、`target_qpos_last` | 3 号保留一秒目标插补；6 号 4310 保留原插补、4340 由 SDK 插补；IK 失败回退行为不同，见 [RUNBOOK](RUNBOOK.md) |
| 真机 Joy-Con 位置插补与 IK 失败独立控制 | `examples/6_ReBot_B601_DM_Joycon_Gpos_REAL.py`：`Example6Controller.step`、`last_sent`；SDK `GRIPPER_MAX_VELOCITY_RAD_S` | LeRobot `test_rebot_example6.py`：双向插补、换向、限位与长停顿、失败 IK 下六轴保持、SDK 位置增益一致性 |
| 手柄夹爪张不开 / 6 号预览张开后回退 | SDK `_send_joint_command`、`gripper_max_velocity_rad_s`；6 号 `last_sent` | 四型号离线联调通过：4310 原插补、4340 2 rad/s、完整行程、保护与预览一致性；不恢复发布目录测试文件；实机开口待验证 |
| 按 0 回零限速 | 1/2 `homing`；4/5 `home_done`；SDK `Go_Home_Pose(max_velocity=...)` | 历史离线回归：仿真连续目标、七路限速、长停顿、保护、回中未完成与超时清理；0.35 rad/s 目标插补，3/6 无 0 键 |
| 4/5 号启动与回中 | `examples/4_ReBot_B601_DM_Key_qpos_REAL.py`、`5_ReBot_B601_DM_Key_Gpos_REAL.py`：`home_cycle`；6 号 `STARTUP_HOME_CYCLES` | 历史离线回归：4 号启动保留固定轮数；4 号按 0、5 号全部回中等待限速目标完成，30 秒超时失能 |
| 真机示例退出回中 | `examples/4_ReBot_B601_DM_Key_qpos_REAL.py`：最外层 `finally` | 正常结束 100 次回中，异常直接失能；6 号正常/中断/异常回归见 [RUNBOOK](RUNBOOK.md) |
| 5 号间歇发包 / 夹爪卡顿 | `examples/5_ReBot_B601_DM_Key_Gpos_REAL.py`：变化时 IK、失败回退、无条件控制发送 | 历史离线回归：IK 失败时夹爪更新、真实 IK 小步、回中复位、异常清理 |
| SDK 电机端位置闭环 / 启动夹爪振荡诊断 | `examples/ReBot_SDK.py`：`_send_joint_command`、`GRIPPER_KP/KD`；参考 BoxArm SDK | 历史离线回归：MIT 位置/增益解码、双方向端点、回中、超时失能；LeRobot 共享参数对照 |
| MIT / 位置速度 / 夹爪保护方案选择 | `docs/research/gripper_control.md`：2026-10-20 MIT 力矩专项检索与方案比较；`Motor_CAN.py`：`control_pos_force`、`change_limit_param` | 源码与厂商手册核对；本机固件/限流参数尚未读取验证 |
| 自动运行日志 | `examples/runtime_log.py`；examples 0–6 入口；共享 SDK；4 号按键事件 | 历史离线回归：子进程异常落盘、退出排空、逐次 SDK 数据快照 |
| 示例硬件零位与夹爪行程 | `examples/0_calibrate_rebot.py`：`ReBotCalibration`、`LIMITS_DEG`；SDK 直接使用硬件反馈 | 历史离线回归；闭合 0、开合行程、回中及无标定文件启动 |

历史过程见 [RUNBOOK](RUNBOOK.md)；当前运行步骤以根目录及 Learning_Model 的双语 README 为准。
