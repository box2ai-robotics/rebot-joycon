#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from .motor_family import JOINT_NAMES, MOTOR_PROFILES, ArmControlMode, GripperControlMode, MotorFamily


@dataclass
class RebotB601FollowerConfig:
    """Configuration shared by the Damiao and RobStride B601 follower arms."""

    # Serial device or native CAN channel.
    port: str

    # Legacy MotorBridge-only software calibration; examples require None.
    software_calibration: Path | None = None
    kinematics_path: Path | None = None
    # auto: hardware-zero SDK for DM, MotorBridge for other motor families.
    control_backend: str = "auto"
    examples_path: Path | None = None

    # Examples use SDK home; MotorBridge uses home only with software_calibration.
    home_on_connect: bool = True
    home_kp_scale: float = 0.8
    home_tolerance_rad: float = 0.1
    home_timeout_s: float = 15.0

    # Motor family: "dm" or "rs".
    motor_family: MotorFamily = MotorFamily.DM

    # CAN transport: "damiao" (serial bridge) or "socketcan" (native CAN).
    can_adapter: str | None = None

    # Damiao serial bridge baud rate.
    dm_serial_baud: int = 921600

    # Disable motor torque before disconnecting.
    disable_torque_on_disconnect: bool = True

    # Maximum position change per command in degrees. None disables the limit.
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Joint name to (send ID, receive ID).
    motor_can_ids: dict[str, tuple[int, int]] | None = None

    # Arm mode: "mit" or "pos_vel".
    control_mode: ArmControlMode = ArmControlMode.MIT

    # Gripper mode: "mit", "force_pos", or "mit_impedance".
    gripper_control_mode: GripperControlMode | None = None

    # MIT gains shared by all joints or keyed by joint name.
    mit_kp: float | dict[str, float] | None = None
    mit_kd: float | dict[str, float] | None = None

    # POS_VEL and FORCE_POS speed limit in degrees per second.
    pos_vel_velocity: float | dict[str, float] | None = None

    # FORCE_POS gripper force as a fraction of peak torque.
    gripper_torque_ratio: float | None = None

    # Impedance gripper moving and holding torque limits in N.m.
    gripper_torque_limit: float | None = None
    gripper_hold_torque_limit: float | None = None

    # Soft limits in public joint degrees.
    joint_limits: dict[str, tuple[float, float]] | None = None

    def __post_init__(self) -> None:
        self.motor_family = MotorFamily(self.motor_family)
        self.control_mode = ArmControlMode(self.control_mode)
        if self.control_backend == "auto":
            self.control_backend = "examples" if self.motor_family is MotorFamily.DM else "motorbridge"
        if self.control_backend not in ("examples", "motorbridge"):
            raise ValueError("control_backend must be auto, examples or motorbridge")
        if self.control_backend == "examples":
            if self.software_calibration is not None:
                raise ValueError("Examples now use motor Flash zero; remove software_calibration")
            if self.motor_family is not MotorFamily.DM:
                raise ValueError("Example 6 requires DM")
            if self.max_relative_target is not None or self.kinematics_path is not None:
                raise ValueError(
                    "Example 6 uses its own model/limits; remove max_relative_target/kinematics_path"
                )
        if self.control_backend == "examples":
            # Keep these unset when serializing config; the SDK remains their single source.
            overrides = [
                name
                for name in (
                    "mit_kp",
                    "mit_kd",
                    "pos_vel_velocity",
                    "joint_limits",
                    "motor_can_ids",
                    "gripper_torque_ratio",
                    "gripper_torque_limit",
                    "gripper_hold_torque_limit",
                )
                if getattr(self, name) is not None
            ]
            if (
                overrides
                or self.control_mode is not ArmControlMode.MIT
                or self.gripper_control_mode not in (None, GripperControlMode.MIT)
            ):
                raise ValueError(
                    "Example 6 reads motor parameters directly from ReBot_SDK.py; remove robot control overrides"
                )
            if self.home_kp_scale != 0.8 or self.home_tolerance_rad != 0.1 or self.home_timeout_s != 15.0:
                raise ValueError(
                    "Example 6 uses SDK home gains and frame-count startup; remove legacy home overrides"
                )
            if (
                self.can_adapter not in (None, "damiao")
                or self.dm_serial_baud != 921600
                or not self.disable_torque_on_disconnect
            ):
                raise ValueError("Example 6 uses its SDK serial transport and disables torque on disconnect")
            return
        profile = MOTOR_PROFILES[self.motor_family]
        joints = tuple(profile.motor_models)
        example_defaults = {}
        if self.software_calibration is not None and self.motor_family is MotorFamily.DM:
            if self.gripper_control_mode is None:
                self.gripper_control_mode = GripperControlMode.MIT
            example_defaults = {
                "mit_kp": dict(zip(JOINT_NAMES, [100.0, 100.0, 100.0, 24.0, 24.0, 24.0, 24.0], strict=True)),
                "mit_kd": dict(zip(JOINT_NAMES, [5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 1.0], strict=True)),
            }
        for name in ("home_kp_scale", "home_tolerance_rad", "home_timeout_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.home_kp_scale > 1:
            raise ValueError("home_kp_scale must not exceed 1")

        if self.can_adapter is None:
            self.can_adapter = profile.can_adapter

        if self.motor_can_ids is None:
            self.motor_can_ids = dict(profile.motor_can_ids)

        if self.gripper_control_mode is None:
            self.gripper_control_mode = profile.gripper_control_mode
        else:
            self.gripper_control_mode = GripperControlMode(self.gripper_control_mode)

        for name in ("mit_kp", "mit_kd", "pos_vel_velocity"):
            value = getattr(self, name)
            default = example_defaults.get(name, getattr(profile, name))
            if value is None:
                value = default
            if value is not None:
                if isinstance(value, (int, float)):
                    value = dict.fromkeys(joints, float(value))
                else:
                    value = {**(default or {}), **value}
                setattr(self, name, value)

        if self.software_calibration is not None:
            # Calibrated limits are filled from the model/JSON by the robot; native-frame
            # defaults must not silently narrow this different coordinate frame.
            self.joint_limits = dict(self.joint_limits or {})
        elif self.joint_limits is None:
            self.joint_limits = dict(profile.joint_limits)
        else:
            self.joint_limits = {**profile.joint_limits, **self.joint_limits}

        if self.gripper_torque_ratio is None:
            self.gripper_torque_ratio = profile.gripper_torque_ratio

        for name in ("gripper_torque_limit", "gripper_hold_torque_limit"):
            if getattr(self, name) is None:
                setattr(self, name, getattr(profile, name))

        if self.gripper_control_mode is GripperControlMode.MIT:
            if self.gripper_torque_limit is None:
                self.gripper_torque_limit = 3.0
            if not math.isfinite(self.gripper_torque_limit) or self.gripper_torque_limit <= 0:
                raise ValueError("gripper_torque_limit must be positive and finite")


@RobotConfig.register_subclass("rebot_b601_follower")
@dataclass
class RebotB601FollowerRobotConfig(RobotConfig, RebotB601FollowerConfig):
    """Registered configuration for the reBot B601 follower robot."""

    def __post_init__(self) -> None:
        RobotConfig.__post_init__(self)
        RebotB601FollowerConfig.__post_init__(self)
