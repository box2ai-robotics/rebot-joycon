# LeRobot: data collection, ACT training, and inference

[English](README.md) | [简体中文](README_zh.md)

Collect demonstrations with a right Joy-Con and the reBot B601-DM, then train and run an ACT policy. Hardware control reuses the root project's example 6 and SDK.

## Installation

Requires Linux, Python 3.12, and `uv`. Keep the root `examples/` and `URDF/` directories. Use the adapted `lerobot/` checkout supplied with this project.

From the **Rebot-Joycon repository root**, enter the learning environment directory:

```bash
cd Learning_Model/lerobot
```

Use the same root `.venv` as the examples. Run the following there; skip environment creation if the root `.venv` already exists:

```bash
uv venv --python 3.12 ../../.venv
uv pip install --python ../../.venv/bin/python -r ../../requirements.txt -e '.[rebot-joycon,hardware,training,test]'
uv pip check --python ../../.venv/bin/python
```

If needed, install `uv` with `python3 -m pip install --user uv` and add it to PATH. NumPy follows LeRobot's `>=2.0,<2.3` requirement. Do not add `all/aloha/hilserl/libero/metaworld` extras. Existing `.venv-rebot` environments are no longer used; keep them until the shared environment has been verified.

**Only Joy-Con teleoperation/recording needs joycon-robotics.** Download it to the repository root and follow its [installation and pairing guide](../joycon-robotics/README.md) ([online](https://github.com/box2ai-robotics/joycon-robotics/blob/master/README.md)). Install its Python package into this environment, from `Learning_Model/lerobot`:

```bash
uv pip install --python ../../.venv/bin/python -e ../../joycon-robotics
```

All commands below run from **`Learning_Model/lerobot`**, except the policy launcher explicitly marked as running from the repository root.

## Offline checks

```bash
../../.venv/bin/python -m pytest tests/teleoperators/test_joycon.py \
  tests/teleoperators/test_rebot_example6.py \
  tests/robots/test_rebot_b601_follower.py -q
```

Uses simulated devices. Recorded validation results are in [RUNBOOK](../docs/RUNBOOK.md).

## Teleoperation

Connect the right Joy-Con and ensure serial/HID permissions and a graphical desktop are available. The serial port must be free. DM defaults to the examples backend and motor Flash zero. No local calibration receipt or motor SN is required. See [calibration](../README.md#calibration) when recalibration is needed.

**Teleoperation and recording immediately enable the arm, send a home target (TCP Z = 0.25 m, gripper closed), and open MuJoCo.** Adjust `/dev/ttyACM0` for your hardware.

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

`ZR` toggles the gripper. Normal completion returns home then disables the arm; `Ctrl+C`, exceptions, or closing the viewer during recording disables it directly. See [logs and protection](../README.md#logs-and-protection).

## Record demonstrations

The example records **10 episodes of up to 200 seconds**, with 5 seconds between episodes to reset the scene manually. Camera: `front`, 640×480, MJPG, 30 FPS; adjust the camera index as needed.

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

Data stays in `data/rebot_smoke` and is not uploaded. To continue an existing dataset, add `--resume=true`; for a new one, change both `repo_id` and `root`. The task string describes "pick up an object and place it in the target area"; keep it consistent with the inference launcher's `TASK`.

| Button | Recording | Reset interval |
| --- | --- | --- |
| `A` | End the episode early. | Finish reset, save, and continue. |
| `Y` | Discard and retry the current episode. | Discard and retry the previous unsaved episode. |

`Y` does not delete saved episodes. Actions and states contain seven angles in degrees; the image field is `observation.images.front`.

## Train ACT

Train for 2,000 steps, saving every 500 steps. No hardware connection is needed:

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

Without CUDA, use `--policy.device=cpu`. This command disables pretrained visual weights, model uploads, and W&B. Use a new output directory if one already exists. The resulting model is `outputs/train/rebot_smoke/checkpoints/last/pretrained_model`; short runs validate the workflow, not task proficiency.

## Run a policy

Edit [run_rebot_policy.py](run_rebot_policy.py): set `POLICY_PATH`, `PORT`, `CAMERA_INDEX`, and `DEVICE`. Keep the full `pretrained_model` directory. From the **Rebot-Joycon repository root**, run:

```bash
python3 Learning_Model/run_rebot_policy.py
```

Defaults: the training output above, `/dev/ttyACM0`, camera `0`, CUDA, and 10-second rollouts. The launcher checks required files before starting. Inference does not need a Joy-Con; match camera name, view, resolution, calibration, and task to recording.

**Startup enables the arm and sends the home target (TCP Z = 0.25 m, gripper closed), then waits for input.**

| Command | Action |
| --- | --- |
| `/start` | Run a policy segment; can be repeated. Does not reset the scene. |
| `/reset` | Stop the policy and return to joint positions recorded at startup; moves the arm. |
| `/stop` | Exit and disable, without returning home. |

[Integration details](lerobot/docs/REBOT_JOYCON.md) · [Validation history](../docs/RUNBOOK.md)
