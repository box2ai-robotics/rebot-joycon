# 任务检索地图

路径相对当前项目根；只列本次核实入口。

| 任务 | 权威文件 / 符号 | 直接验证 |
| --- | --- | --- |
| 直接运行示例 / 已有策略 | [零位校准](../README_zh.md#零位校准与使用)；[运行示例](../README_zh.md#运行)：1–6 号独立命令及各控制方式按键；[运行策略](../Learning_Model/README_zh.md#运行策略)；`Learning_Model/run_rebot_policy.py`：`main`、`POLICY_PATH` | README 命令 Bash 语法、入口路径、按键映射和新增本地链接静态检查；推理需已有完整模型，脚本检查缺失文件后停止 |
| 遥操启动缺少 `teleop` / `robot` 配置 | [真机遥操](../Learning_Model/README_zh.md#真机遥操)的完整命令；`lerobot_teleoperate.TeleoperateConfig` | Bash 语法检查与仅配置解析；未定义 `REBOT_ARGS` 时展开为空，复现缺字段错误；不启动设备 |
| LeRobot 离线自检回中报 `Invalid gripper command or torque feedback` | `Learning_Model/lerobot/tests/teleoperators/test_joycon.py`：`test_calibrated_robot_home_gains_and_timeout`；follower `_send_gripper` | 模拟反馈须含 `pos/vel/torq`；六轴检查电机端增益，MotorBridge 夹爪检查主机 PD 限幅力矩；三文件自检命令与 136 项通过记录见 [RUNBOOK](RUNBOOK.md) |
| LeRobot 相机 MJPG 采集格式 | `Learning_Model/run_rebot_policy.py`：`cameras`；`Learning_Model/README.md`：相机命令参数；LeRobot `OpenCVCamera._validate_fourcc` | 入口配置及文档 JSON 静态核对；保持 640×480、30 FPS；相机实测待验证 |
| 全部实机夹爪反馈超过 1 N·m 收回目标 | `examples/gripper_protection.py`：`GripperTorqueProtection.apply`、`GRIPPER_TORQUE_TRIP_NM`；SDK `_protect_gripper_target`；LeRobot follower `_send_gripper` | 根测试：正负超限、锁存、反向卸力、键盘无目标积累；LeRobot follower/example6 测试：多模式、双臂隔离、修正动作返回与插补 |
| 仿真 Joy-Con 夹爪开合 | `examples/3_ReBot_B601_DM_Joycon_Gpos_SIM.py`：`GRIPPER_TRAVEL_TIME_S`、`gripper_dt`、`target_qpos_last` | 3/6 号均恢复一秒目标插补；IK 失败回退行为不同，见 [RUNBOOK](RUNBOOK.md) |
| 真机 Joy-Con 位置插补与 IK 失败独立控制 | `examples/6_ReBot_B601_DM_Joycon_Gpos_REAL.py`：`GRIPPER_TRAVEL_TIME_S`、`Example6Controller.step`、`last_sent` | LeRobot `test_rebot_example6.py`：双向插补、换向、限位与长停顿、失败 IK 下六轴保持、SDK 位置增益一致性 |
| 4/5 号对齐 Learning_Model 启动 | `examples/4_ReBot_B601_DM_Key_qpos_REAL.py`、`5_ReBot_B601_DM_Key_Gpos_REAL.py`：`home_cycle`；6 号 `STARTUP_HOME_CYCLES` | `examples/test_rebot_keyboard_real.py`：固定轮数、不按反馈误差阻塞、时间步长、释放及复位 |
| 真机示例退出回中 | `examples/4_ReBot_B601_DM_Key_qpos_REAL.py`：最外层 `finally` | 正常结束 100 次回中，异常直接失能；6 号正常/中断/异常回归见 [RUNBOOK](RUNBOOK.md) |
| 5 号间歇发包 / 夹爪卡顿 | `examples/5_ReBot_B601_DM_Key_Gpos_REAL.py`：变化时 IK、失败回退、无条件控制发送 | 同测试：IK 失败时夹爪更新、真实 IK 小步、回中复位、异常清理 |
| SDK 电机端位置闭环 / 启动夹爪振荡诊断 | `examples/ReBot_SDK.py`：`_send_joint_command`、`GRIPPER_KP/KD`；参考 BoxArm SDK | `test_rebot_keyboard_real.py`：MIT 位置/增益解码、双方向端点、回中、超时失能；LeRobot 共享参数对照 |
| MIT / 位置速度 / 夹爪保护方案选择 | `docs/research/gripper_control.md`：2026-10-20 MIT 力矩专项检索与方案比较；`Motor_CAN.py`：`control_pos_force`、`change_limit_param` | 源码与厂商手册核对；本机固件/限流参数尚未读取验证 |
| 自动运行日志 | `examples/runtime_log.py`；examples 0–6 入口；共享 SDK；4 号按键事件 | `examples/test_runtime_log.py`：子进程异常落盘、退出排空、逐次 SDK 数据快照 |
| 所有示例夹爪无法闭合 / 端点 | `examples/0_calibrate_rebot.py`：`ReBotCalibration`；`examples/calibration/rebot.json`、`rebot.encoders.json` | 同测试：正的实测闭合角不被名义 0 裁剪、正反方向、无效端点拒绝、4/5 完整开合；行程由加载的端点之差计算 |

历史过程见 [RUNBOOK](RUNBOOK.md)；当前运行步骤以根目录及 Learning_Model 的双语 README 为准。
