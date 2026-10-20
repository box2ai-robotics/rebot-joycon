"""Right Joy-Con Cartesian teleoperation for the calibrated B601-DM."""

import math
from dataclasses import dataclass
from pathlib import Path

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("joycon")
@dataclass
class JoyconTeleopConfig(TeleoperatorConfig):
    use_example6: bool = True
    examples_path: Path | None = None
    kinematics_path: Path | None = None
    software_calibration: Path | None = None
    xyz_scale: float = 2 / 3
    rotation_scale: float = 1.0
    gripper_speed_rad_s: float | None = None
    max_joint_step_deg: float | None = None

    def __post_init__(self) -> None:
        for name in ("xyz_scale", "rotation_scale", "gripper_speed_rad_s", "max_joint_step_deg"):
            value = getattr(self, name)
            if value is None and name in ("xyz_scale", "rotation_scale"):
                raise ValueError(f"{name} must be positive and finite")
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be positive and finite")
