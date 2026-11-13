# reBot B601-DM：Joy-Con 采集与 ACT 训练

核对日期：2026-10-17。由 Box2AI 在当前 LeRobot 中接入；本轮仅做离线验证，不启动真机、相机、真实手柄或正式训练，不上传数据。

## BoxArm 是如何适配的

本地参考目录：`/home/boxjod/Workspace/BoxArm/Learning_Model/lerobot`。

| 层 | BoxArm 本地实现 | 本次 reBot 实现 |
| --- | --- | --- |
| 配置注册 | `teleoperators/joycon/configuration_joycon.py` 注册 `joycon` | [`JoyconTeleopConfig`](../src/lerobot/teleoperators/joycon/config_joycon.py) 注册同名设备类型 |
| 手柄与 IK | `joycon_controller.py` 获取末端目标，相对导入同目录复制的 `BoxArm_Kinematics.py`，维护 MuJoCo 关节种子 | [`JoyconTeleop`](../src/lerobot/teleoperators/joycon/teleop_joycon.py) 默认启动外层示例 6 的同一个 `Example6Controller`，使用完全相同的 MuJoCo 步进状态作为 IK 种子 |
| 输出动作 | 七个臂关节＋`Hand_State`，共八路 `.pos` | 六轴＋夹爪，共七路 `.pos`，与原生 reBot Robot 的关节名一致，单位为度 |
| Robot | `BoxArm_follower/boxarm_follower.py` 包装 BoxArm SDK、手部 SDK 与相机 | 复用 [`rebot_b601_follower`](../src/lerobot/robots/rebot_b601_follower/rebot_b601_follower.py) ：软件校准模式默认直接包装外层 SDK/Motor_CAN；原生 MotorBridge 作为可选旧后端保留 |
| 采集/训练 | 录制关节观测、动作、图像，策略学习动作 | 使用标准 `lerobot-record`、`lerobot-train`，无需给 ACT 加 IK 层 |

以上是本地源码事实，未运行 BoxArm 硬件。参考 Joy-Con 类中 `is_connected` 固定为 False，`action_features` 引用了未定义的 `self.arm`，控制线程也没有停止条件；因此本次沿用接口结构，不直接复制这些实现。

## 坐标、校准和控制行为

- 动作/状态顺序固定：`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, gripper`，键名带 `.pos`，七项均为 **度**。运动学内部使用 **m/rad**。
- 0 号将七路机械零位写入电机 Flash；运行直接使用电机内部零位，不再要求本地完成记录、CAN/SN 绑定文件或软件偏移文件。
- 新机械臂建议先按规定机械姿态校准；已有硬件零位无需为生成本地文件再次校准。断电后可在同姿态执行 `--verify-hardware-zero`，仅查询电机，不写文件。
- DM 的 `auto` 默认选择 `examples`，直接加载外层示例 6 和共享 SDK。`software_calibration` 必须省略；显式旧 DM MotorBridge 连接被拒绝，实机统一使用共享 SDK 的硬件零位和夹爪适配。RS 后端不受影响。
- 同一 SDK 负责电机类型、MIT 编码/收发、硬件零位、增益衰减和限位；LeRobot 不再重复实现这些逻辑。手柄所有显式/隐式参数、MuJoCo 模型、两次步进、IK 种子、前 40 轮回中切换均来自示例 6。完整参数表见 [主操作说明](../../README.md#3-校准文件与真机遥操)。
- `Example6Runtime` 以示例的 0.001 s 目标间隔运行独立控制线程；LeRobot 30 Hz 仅采样状态/相机和已发送动作，不再次发命令。实际频率受计算/串口影响；LeRobot 不显示 MuJoCo 窗口，保留全部仿真步进。
- 手柄、Robot 必须使用相同的 `examples_path`。部署保留外层 `examples`、`URDF` 和手柄库。不需要设备完成记录。
- 正常结束先停止控制线程，再按示例回中 100 次并失能；异常停止不追加回中。A/Y 上升沿由独立线程缓存，采集循环消费，短按不再依赖 30 Hz 的瞬间采样。Y 只丢弃未保存缓冲段。
- 历史 MotorBridge 算法只保留独立测试；DM 实机统一走 examples。其他电机系列仍可使用 MotorBridge。

## 安装

在本目录（`Learning_Model/lerobot`）执行，使用独立 Python 3.12 环境：

```bash
uv venv --python 3.12 .venv-rebot
uv pip install --python .venv-rebot/bin/python -e '.[rebot-joycon,hardware,training]'
uv pip install --python .venv-rebot/bin/python -e ../../joycon-robotics
source .venv-rebot/bin/activate
```

`joycon-robotics` 来自当前项目外层目录，不修改参考 BoxArm 仓库。`rebot-joycon` extra 提供 MuJoCo 3.16.0、Robotics Toolbox/SpatialMath 和旧后端的 MotorBridge 依赖；`training` 提供数据集和训练依赖。不要把早期 examples 的 NumPy 1.x 环境直接当作此 LeRobot 的环境（当前 LeRobot 要求 NumPy 2.x）。

## 采集

连接右 Joy-Con，确认串口与相机索引。下面相机 `0` 是配置示例，需按实际设备修改。命令会使能并控制真机，应在准备好机械臂后由操作者执行。

```bash

python -m lerobot.scripts.lerobot_record \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.motor_family=dm \
  --robot.control_backend=examples \
  --robot.id=rebot \
  --robot.cameras='{"front":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":30}}' \
  --teleop.type=joycon \
  --teleop.use_example6=true \
  --teleop.xyz_scale=0.6666666667 \
  --dataset.repo_id=local/rebot_pick \
  --dataset.root=./data/rebot_pick \
  --dataset.single_task='抓取物体并放到目标区域' \
  --dataset.fps=30 \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=15 \
  --dataset.push_to_hub=false \
  --play_sounds=false
```

仅遥操时使用 `python -m lerobot.scripts.lerobot_teleoperate`，沿用上述 robot/teleop 参数，去掉 dataset/play_sounds 参数。相机也可省略用于联调，但下面 ACT 视觉训练需要至少一幅相机图像。默认直接使用 examples 的 SDK/Motor_CAN，包括原来的 MIT 模式确认流程；不能用“切换到 LeRobot”绕过设备或通信故障。

## ACT 训练

采集完成后执行，训练不连接机械臂，也不调用 Joy-Con/FK/IK。它读取七维校准关节状态、相机图像及七维动作。推理仍输出同一套关节坐标，经相同 Robot 软件校准转换到电机坐标。

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/rebot_pick \
  --dataset.root=./data/rebot_pick \
  --policy.type=act \
  --policy.device=cuda \
  --policy.pretrained_backbone_weights=null \
  --policy.push_to_hub=false \
  --output_dir=./outputs/train/rebot_act \
  --job_name=rebot_act \
  --batch_size=8 \
  --steps=20000 \
  --wandb.enable=false
```

这里关闭预训练视觉权重下载，仅给出可从本地数据启动的配置；没有 GPU 时改 `--policy.device=cpu`。20,000 步只是起始配置，不是对训练收敛或任务成功率的承诺。复用数据训练不同策略时，按该策略的输入要求调整配置即可。部署时保持与采集一致的机械零位；不要混用旧软件归零数据和新硬件零位数据。

## 离线验证及边界

测试入口：[`test_joycon.py`](../tests/teleoperators/test_joycon.py) 与 [`test_rebot_b601_follower.py`](../tests/robots/test_rebot_b601_follower.py)。

```bash
PYTHONPATH=src uv run --no-project --python /tmp/rebot-lerobot-check/bin/python \
  pytest tests/teleoperators/test_rebot_example6.py tests/teleoperators/test_joycon.py tests/robots/test_rebot_b601_follower.py -q
```

`/tmp/rebot-lerobot-check` 是本轮验证环境，普通用户可改用安装好的 `.venv-rebot/bin/python`。

验证结果：LeRobot 相关测试 **78 项通过**，外层 examples 回归 **19 项通过**；Ruff 检查、`uv lock --check`、采集/训练 CLI 配置解析及本文本地链接检查通过。新增依赖已同步 `uv.lock`。

已验证：配置注册、真实示例 IK 的 2 mm 平移、模型同步、失败保持、关节限制、NaN 拒绝、HID 清理、校准转换、不写硬件零位、旧后端 MIT 参数与回中成功/超时、A/Y 防长按及采集循环集成，以及原生 DM/RS 回归。离线采集实际落盘后完成 ACT 单批 CPU 前向/反向。

新增验证：保存修改前示例 6 的原循环作为对照，在模拟 HID/SDK 下与共用实现逐步比较 45 轮，MuJoCo qpos、关节目标、回中/控制发送序列一致；独立线程无需采集调用即可运行，A/Y 短按被缓存，异常触发失能，默认 Robot 直接使用原 SDK，采集读取不重复发包。

真实 Joy-Con、CAN 反馈时效、相机采集、长期闭环、正式策略训练与真机推理尚未验证。按时间生成目标不等于保证电机实测速度，仍受增益、负载和限位影响。

## 2026-10-17：按 BoxArm 方式内置运动学

已确认 BoxArm 仓库中的 `teleoperators/joycon/BoxArm_Kinematics.py` 是仓库内的普通 Python 文件，控制器使用 `from .BoxArm_Kinematics import ...`。早期适配已将外层 reBot 的 DH、TCP、FK/IK 和回中求解代码复制到 LeRobot 同层目录。该副本现在用于显式旧后端；当前默认通过共用示例 6 直接加载外层源码。校准读取逻辑从 `0_calibrate_rebot.py` 一并复制，不复制串口采样入口或设备专属标定数据。

保留源文件版权信息和复制来源；后续修改外层运动学时应同步这一指定副本，并运行 `test_bundled_model_matches_original_and_needs_no_external_code`：核对 DH/变换常量、100 组姿态 FK 和回中解。测试还验证纯模型构建不读外部文件、默认加载与外部覆盖互不混淆。

## 首次测试顺序（2026-10-17）

1. 按安装章节创建 `.venv-rebot`，额外安装测试依赖：`uv pip install --python .venv-rebot/bin/python -e '.[test]'`。先运行本页三组 pytest，预期 78 项通过，不需要连接设备。
2. 先不配置相机，用下列命令测试 60 秒真机遥操。开始前退出其他串口控制程序并连接右 Joy-Con。这个适配先回到 examples 的 Z=0.25 m 初始位姿，再按固定参考控制；ZR 切换夹爪，Ctrl+C 提前退出。

```bash
python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.motor_family=dm \
  --robot.control_backend=examples \
  --robot.id=rebot \
  --teleop.type=joycon \
  --teleop.use_example6=true \
  --fps=30 --teleop_time_s=60
```

3. 遥操正常后用采集章节的命令，首次将任务数据路径换为 `local/rebot_smoke` / `./data/rebot_smoke`，设置 `num_episodes=2`、`episode_time_s=10`、`reset_time_s=5`。相机索引按实际设备修改；这一阶段必须保留图像才能使用下面的 ACT 视觉训练。
4. 短训练仅验证数据和训练流程，使用相同测试数据集，执行 100 步：

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/rebot_smoke \
  --dataset.root=./data/rebot_smoke \
  --policy.type=act --policy.device=cuda \
  --policy.pretrained_backbone_weights=null \
  --policy.push_to_hub=false \
  --output_dir=./outputs/train/rebot_smoke \
  --job_name=rebot_smoke \
  --batch_size=2 --num_workers=0 \
  --steps=100 --save_freq=100 --log_freq=10 \
  --wandb.enable=false
```

完成条件：loss 为有限值、训练正常完成并在输出目录生成 checkpoint。100 步不代表策略已经学会任务；真机策略测试须另行准备，不能直接把这份短训练模型当成有效策略。没有 CUDA 可改 `--policy.device=cpu`。重复测试应更换数据/输出目录，或按 LeRobot 的 resume 流程恢复，不要覆盖已有采集数据。本节真机与 100 步训练命令尚未实机执行，现有证据仍为离线 78 项测试和 ACT 单批前向/反向。
