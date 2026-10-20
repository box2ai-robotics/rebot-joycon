"""Load the project's authoritative example 6 without opening any devices."""

import importlib
import importlib.util
import sys
from pathlib import Path
from types import ModuleType


def example_directory(path: Path | None = None) -> Path:
    directory = (path or Path(__file__).resolve().parents[5] / "examples").expanduser().resolve()
    if not (directory / "6_ReBot_B601_DM_Joycon_Gpos_REAL.py").is_file():
        raise FileNotFoundError(f"Example 6 not found in {directory}; set examples_path")
    return directory


def load_example6(path: Path | None = None) -> ModuleType:
    directory = example_directory(path)
    name = "_lerobot_rebot_example6"
    cached = sys.modules.get(name)
    if cached is not None:
        if Path(cached.__file__).parent != directory:
            raise RuntimeError("Example 6 already loaded from another directory")
        return cached
    # These scripts import siblings by their original names. Refuse a foreign SDK/model.
    for module_name in ("Motor_CAN", "0_calibrate_rebot", "ReBot_Kinematics", "gripper_protection", "ReBot_SDK"):
        existing = sys.modules.get(module_name)
        if existing is not None and Path(existing.__file__).resolve() != directory / f"{module_name}.py":
            raise RuntimeError(f"{module_name} is already loaded from another project")
    sys.path.insert(0, str(directory))
    try:
        for module_name in ("Motor_CAN", "0_calibrate_rebot", "ReBot_Kinematics", "ReBot_SDK"):
            importlib.import_module(module_name)
        spec = importlib.util.spec_from_file_location(name, directory / "6_ReBot_B601_DM_Joycon_Gpos_REAL.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules[name] = module
        return module
    finally:
        sys.path.pop(0)


def load_gripper_protection(path: Path | None = None) -> ModuleType:
    """Load the same protection as the SDK without importing hardware/GUI code."""
    source = example_directory(path) / "gripper_protection.py"
    name = "gripper_protection"
    cached = sys.modules.get(name)
    if cached is not None:
        if Path(cached.__file__).resolve() != source:
            raise RuntimeError("Gripper protection already loaded from another project")
        return cached
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[name] = module
    return module
