# Rebot-Joycon

[English](README.md) | [简体中文](README_zh.md) · [MIT License](LICENSE)

A codebase for controlling robots with Joy-Cons to collect data for embodied AI.

Supports keyboard and right Joy-Con control of the **reBot B601-DM**, in MuJoCo and on hardware. For data collection, ACT training, and inference, see [Learning_Model](Learning_Model/README.md).

## Installation

Requires Linux, Python 3.12, a graphical desktop, OpenGL, and X11/Xorg keyboard input. Run commands from the repository root:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Examples and Learning_Model share this root `.venv`, using LeRobot's NumPy 2.x range. Skip environment creation if it already exists; add learning dependencies using the [Learning_Model installation](Learning_Model/README.md#installation).

**Keyboard control does not require joycon-robotics.** Only for Joy-Con control (examples 3/6), download and install the optional dependency:

```bash
git clone https://github.com/box2ai-robotics/joycon-robotics.git joycon-robotics
.venv/bin/python -m pip install -e ./joycon-robotics
```

Skip cloning if the directory already exists. For drivers, Bluetooth pairing, and troubleshooting, follow [joycon-robotics/README.md](joycon-robotics/README.md) ([online copy](https://github.com/box2ai-robotics/joycon-robotics/blob/master/README.md)). Use the same `.venv` for its Python installation.

## Run

Run `.venv/bin/python examples/<filename>` with a file below:

| Control | Simulation | Real hardware |
| --- | --- | --- |
| Keyboard joints | `1_ReBot_B601_DM_Key_qpos_SIM.py` | `4_ReBot_B601_DM_Key_qpos_REAL.py` |
| Keyboard end-effector IK | `2_ReBot_B601_DM_Key_Gpos_SIM.py` | `5_ReBot_B601_DM_Key_Gpos_REAL.py` |
| Right Joy-Con | `3_ReBot_B601_DM_Joycon_Gpos_SIM.py` | `6_ReBot_B601_DM_Joycon_Gpos_REAL.py` |

For example, keyboard translation/rotation:

```bash
.venv/bin/python examples/2_ReBot_B601_DM_Key_Gpos_SIM.py
```

**Examples 4–6 immediately enable the arm and send a home target (TCP Z = 0.25 m, gripper closed).** They detect `/dev/ttyACM0` or `/dev/ttyACM1` using serial string `0000000`; serial access is required. All examples open a MuJoCo window.

| Mode | Keys |
| --- | --- |
| Joint control (1/4) | `1…6` increase; `q/w/e/r/t/y` decrease; `7/u` close/open gripper. |
| Translation (2/5) | X: `w/s`; Y: `a/d`; Z: `r/f` (positive/negative). |
| Rotation (2/5) | Roll: `e/q`; pitch: `g/t`; yaw: `z/c` (positive/negative). |
| Gripper (2/5) | `1/3` close/open. |
| Keyboard modes | Hold to update the target; release to stop updating; `0` returns home. |
| Joy-Con (3/6) | `ZR` toggles the gripper with one-second full-travel target interpolation. |

Closing the window normally returns the real arm home before disabling it. `Ctrl+C` or an exception disables it without an additional return-home motion.

<a id="零位校准与使用"></a>

## Calibration

To recalibrate, place the arm at mechanical zero with the gripper fully closed:

<img src="docs/images/b601dm_zeroposition.jpg" alt="Mechanical zero pose, gripper closed" width="50%">

```bash
.venv/bin/python examples/0_calibrate_rebot.py --port /dev/ttyACM0
```

This reads feedback without enabling motors or commanding motion. Keep both files in [examples/calibration](examples/calibration): `rebot.json` and `rebot.encoders.json`. They are shared with LeRobot and must match your arm.

## Directory guide

| Path | Purpose |
| --- | --- |
| `examples/` | Examples 0–6, shared SDK, kinematics, motor communication, protection, and tests. |
| `URDF/` | Robot models, meshes, and validation records. |
| `joycon-robotics/` | Optional external Joy-Con dependency; ignored by Git. |
| `Learning_Model/` | Adapted LeRobot, data collection, ACT training, and inference. |
| `gravity_compensation/` | Gravity compensation demo; needs external reBot packages and Pinocchio. |
| `docs/` | [Project map](docs/CODEX_PROJECT_MAP.md), [validation history](docs/RUNBOOK.md), and reference images. |
| `logs/` | Automatic runtime logs; kept locally and ignored by Git. |

<a id="自动运行日志"></a>

## Logs and protection

Examples 0–6 and SDK hardware connections retain console and event logs in `logs/`. Real-arm gripper protection retracts the target and latches the blocked direction when absolute feedback torque exceeds 1 N·m; reverse movement is allowed. This is not a hard motor torque limit. Simulation has no motor torque feedback.

Offline checks: `.venv/bin/python -m unittest discover -s examples -p 'test_*.py' -q`.

Third-party code and models retain their own licenses. `Learning_Model/lerobot` is included as ordinary source files with local reBot/Joy-Con adaptations, based on [LeRobot commit 53feaef](https://github.com/huggingface/lerobot/commit/53feaef37b67e8f4c23b6953e880ff535e39d6fe). An unmodified upstream clone is not a replacement.
