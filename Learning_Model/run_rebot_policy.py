"""启动 ReBot 策略推理；路径相对本文件，不依赖终端当前目录。"""

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
LEROBOT = ROOT / "lerobot"
PYTHON = ROOT.parent / ".venv/bin/python"

# 按实际模型和设备修改这里。当前 100 步模型仅用于流程验证。
POLICY_PATH = LEROBOT / "outputs/train/rebot_smoke/checkpoints/last/pretrained_model"
CALIBRATION = ROOT.parent / "examples/calibration/rebot.json"
PORT = "/dev/ttyACM0"
CAMERA_INDEX = 0
DEVICE = "cuda"
TASK = "抓取物体并放到目标区域"
DURATION = 10


def main() -> int:
    required = [PYTHON, CALIBRATION] + [
        POLICY_PATH / name
        for name in (
            "config.json",
            "model.safetensors",
            "policy_preprocessor.json",
            "policy_postprocessor.json",
        )
    ]
    missing = [path for path in required if not path.is_file()]
    if missing:
        for path in missing:
            print(f"文件缺失：{path}", file=sys.stderr)
        print("未启动推理，请检查文件顶部的路径配置。", file=sys.stderr)
        return 1

    cameras = {"front": {"type": "opencv", "index_or_path": CAMERA_INDEX,
                         "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"}}
    command = [
        str(PYTHON), "-m", "lerobot.scripts.lerobot_rollout",
        "--strategy.type=base",
        "--inference.type=sync",
        f"--policy.path={POLICY_PATH.resolve()}",
        f"--device={DEVICE}",
        "--robot.type=rebot_b601_follower",
        f"--robot.port={PORT}",
        "--robot.motor_family=dm",
        "--robot.control_backend=examples",
        "--robot.id=rebot",
        f"--robot.software_calibration={CALIBRATION.resolve()}",
        f"--robot.cameras={json.dumps(cameras)}",
        f"--task={TASK}",
        "--fps=30",
        f"--duration={DURATION}",
        "--interactive=true",
        "--return_to_initial_position=false",
        "--play_sounds=false",
    ]
    print("启动时会使能并回中；输入 /start 执行策略，/stop 退出。", flush=True)
    try:
        return subprocess.run(command, cwd=LEROBOT, check=False).returncode
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
