# LeRobot：数据采集、ACT 训练与推理

[English](README.md) | [简体中文](README_zh.md)

使用右 Joy-Con 和 reBot B601-DM 采集示范，训练并运行 ACT 策略。真机控制复用根目录示例 6 与 SDK。

## 安装

需要 Linux、Python 3.12 和 `uv`。保留根目录 `examples/`、`URDF/`，使用本项目适配的 `lerobot/`。

从 **Rebot-Joycon 仓库根目录**进入学习环境目录：

```bash
cd Learning_Model/lerobot
```

与 examples 共用根目录 `.venv`。在该目录执行；根目录已有 `.venv` 时跳过创建：

```bash
uv venv --python 3.12 ../../.venv
uv pip install --python ../../.venv/bin/python -r ../../requirements.txt -e '.[rebot-joycon,hardware,training,test]'
uv pip check --python ../../.venv/bin/python
```

未安装 `uv` 时执行 `python3 -m pip install --user uv`，并确保其在 PATH 中。NumPy 遵循 LeRobot 的 `>=2.0,<2.3` 要求，不要混装 `all/aloha/hilserl/libero/metaworld` extras。原 `.venv-rebot` 不再使用，共享环境验证通过前可保留。

**仅 Joy-Con 遥操和录制需要 joycon-robotics。** 将其下载到仓库根目录，按[手柄安装与配对说明](../joycon-robotics/README_zh.md)（[在线版](https://github.com/box2ai-robotics/joycon-robotics/blob/master/README_zh.md)）配置。在 `Learning_Model/lerobot` 内安装到本环境：

```bash
uv pip install --python ../../.venv/bin/python -e ../../joycon-robotics
```

以下命令均在 **`Learning_Model/lerobot`** 执行，最后的策略启动器除外（明确标注从仓库根目录运行）。

## 离线自检

```bash
../../.venv/bin/python -m pytest tests/teleoperators/test_joycon.py \
  tests/teleoperators/test_rebot_example6.py \
  tests/robots/test_rebot_b601_follower.py -q
```

使用模拟设备；已有验证记录见 [RUNBOOK](../docs/RUNBOOK.md)。

## 真机遥操

先连接右 Joy-Con，确保有串口/HID 权限和图形桌面，串口未被占用。DM 默认使用 examples 后端及电机 Flash 零位，不要求本地完成记录或 SN 检查（无软件零偏）；重新校准见[校准说明](../README_zh.md#零位校准与使用)。

**遥操和采集启动即使能、发送回中目标（TCP Z=0.25 m、夹爪闭合），并打开 MuJoCo。** 按实际设备修改 `/dev/ttyACM0`。

```bash
../../.venv/bin/python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.motor_family=dm \
  --robot.control_backend=examples \
  --robot.id=rebot \
  --teleop.type=joycon \
  --teleop.use_example6=true \
  --fps=30 \
  --teleop_time_s=60
```

`ZR` 切换夹爪。正常完成后回中再失能；`Ctrl+C`、异常或采集中提前关窗直接失能。参阅[日志与保护说明](../README_zh.md#自动运行日志)。

## 采集数据

下面采集 **10 段，每段最多 200 秒**，段间留 5 秒手动复位场景。相机为 `front`、640×480、MJPG、30 FPS，按设备修改索引。

```bash
../../.venv/bin/python -m lerobot.scripts.lerobot_record \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.motor_family=dm \
  --robot.control_backend=examples \
  --robot.id=rebot \
  --teleop.type=joycon \
  --teleop.use_example6=true \
  --robot.cameras='{"front":{"type":"opencv","index_or_path":0,"width":640,"height":480,"fps":30,"fourcc":"MJPG"}}' \
  --dataset.repo_id=local/rebot_smoke \
  --dataset.root=./data/rebot_smoke \
  --dataset.no_stamp=true \
  --dataset.single_task='抓取物体并放到目标区域' \
  --dataset.fps=30 \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=200 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false \
  --play_sounds=true
```

数据保存在 `data/rebot_smoke`，不上传。已有数据追加 `--resume=true` 可继续采集；新数据集同时修改 `repo_id/root`。任务描述与推理启动器的 `TASK` 保持一致。

| 按键 | 录制时 | 复位阶段 |
| --- | --- | --- |
| `A` | 提前结束当前条。 | 结束复位，保存并进入下一条。 |
| `Y` | 丢弃当前条并重录。 | 丢弃尚未保存的上一条并重录。 |

`Y` 不删除已保存条目。动作和状态为七维角度（度），图像字段为 `observation.images.front`。

## ACT 训练

训练 2000 步，每 500 步保存，无需连接设备：

```bash
../../.venv/bin/python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/rebot_smoke \
  --dataset.root=./data/rebot_smoke \
  --policy.type=act \
  --policy.device=cuda \
  --policy.pretrained_backbone_weights=null \
  --policy.push_to_hub=false \
  --output_dir=./outputs/train/rebot_smoke \
  --job_name=rebot_smoke \
  --batch_size=2 \
  --num_workers=0 \
  --steps=2000 \
  --save_freq=500 \
  --log_freq=10 \
  --wandb.enable=false
```

无 CUDA 时改为 `--policy.device=cpu`。命令不下载预训练视觉权重、不上传模型、不启用 W&B。输出目录已存在时换一个新目录。模型位于 `outputs/train/rebot_smoke/checkpoints/last/pretrained_model`；短训练只验证流程，不代表学会任务。

## 运行策略

修改 [run_rebot_policy.py](run_rebot_policy.py) 中的 `POLICY_PATH`、`PORT`、`CAMERA_INDEX`、`DEVICE`，保留完整 `pretrained_model` 目录。从 **Rebot-Joycon 仓库根目录**运行：

```bash
python3 Learning_Model/run_rebot_policy.py
```

默认使用上述训练输出、`/dev/ttyACM0`、相机 `0`、CUDA，每次推理 10 秒。启动器先检查所需文件。推理不需要 Joy-Con；相机名称、视角、分辨率、校准及任务须与采集一致。

**启动即使能、发送回中目标（TCP Z=0.25 m、夹爪闭合），然后等待输入。**

| 指令 | 行为 |
| --- | --- |
| `/start` | 执行一段策略，可重复；不自动复位场景。 |
| `/reset` | 停止策略，回到启动时记录的关节位置，会产生真机运动。 |
| `/stop` | 退出并失能，不自动回位。 |

[适配说明](lerobot/docs/REBOT_JOYCON.md) · [验证历史](../docs/RUNBOOK.md)
