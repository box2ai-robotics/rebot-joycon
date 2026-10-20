# reBot 示例项目约定

- 入口见 [README](README.md)、[任务检索地图](docs/CODEX_PROJECT_MAP.md)；离线验证记录见 [RUNBOOK](docs/RUNBOOK.md)。
- `examples/1–3` 为仿真，`4–6` 会连接并控制机械臂；离线验证不得启动 REAL 主程序。SDK 离线实例须指定 `if_connect_directly=False`，手柄和窗口须替换为模拟接口。
- 4/5/6 与 LeRobot examples 后端共用 `examples/ReBot_SDK.py`。修改共享参数前核对影响范围，区分离线命令验证与真机运动验证。
- 保留校准文件与已有改动；不为文档维护修改第三方源码。修改 LeRobot 子项目时另读其局部约定。
- 运行日志默认长期保留：examples 0–6 和共享 SDK 连接通过 `examples/runtime_log.py` 自动记录至 `logs/`；后续运行入口变更须保持日志接入，不自动删除历史日志。
- 夹爪已按用户回退为电机端 MIT 位置闭环（Kp=2、Kd=1，回中 Kp=1.6），6 号恢复一秒目标插补；已撤销 0/1 直接端点与 1 N·m 纯力矩分支。原 3 N·m 软件限幅未恢复，不能把 `tau=0` 表述为总输出力矩限幅。
- 夹爪新增反馈保护：`abs(torque[6]) > 1 N·m` 将目标收回反馈位置并锁存受阻方向边界，允许反向卸力；不是驱动内部总力矩硬限幅。共享 SDK 回中也受保护，日志记录触发和解除。
- 用户已反馈该保护实机测试成功，要求统一全部实机入口。权威算法/阈值位于 `examples/gripper_protection.py`，SDK 和 LeRobot MotorBridge 共用；4/5 键盘目标须回读 SDK 修正值，防止积累。用户明确保留 1–3 仿真预览，无电机力矩反馈，不伪造力矩保护。
