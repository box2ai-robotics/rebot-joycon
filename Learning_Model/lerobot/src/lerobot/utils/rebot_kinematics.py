"""Load the bundled reBot model, or an explicitly selected external model."""

import importlib
import sys
from pathlib import Path
from types import ModuleType


def load_rebot_kinematics(path: Path | None = None) -> ModuleType:
    if path is None:
        return importlib.import_module("lerobot.teleoperators.joycon.ReBot_Kinematics")
    path = path.expanduser().resolve()
    bundled = Path(__file__).resolve().parents[1] / "teleoperators/joycon/ReBot_Kinematics.py"
    if path == bundled:
        return importlib.import_module("lerobot.teleoperators.joycon.ReBot_Kinematics")
    if not path.is_file():
        raise FileNotFoundError(f"reBot kinematics not found: {path}; set kinematics_path explicitly.")
    # The example imports its sibling 0_calibrate_rebot module.
    for name, expected in (
        ("ReBot_Kinematics", path),
        ("0_calibrate_rebot", path.with_name("0_calibrate_rebot.py")),
    ):
        existing = sys.modules.get(name)
        if existing is not None and Path(existing.__file__).resolve() != expected:
            raise RuntimeError(f"{name} is already loaded from a different project: {existing.__file__}")
    sys.path.insert(0, str(path.parent))
    try:
        return importlib.import_module("ReBot_Kinematics")
    finally:
        sys.path.pop(0)
