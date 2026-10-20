"""Joy-Con → bundled reBot FK/IK → seven joint positions in degrees."""

import logging
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from lerobot.robots.rebot_b601_follower.motor_family import JOINT_NAMES
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.rebot_example6 import example_directory, load_example6
from lerobot.utils.rebot_kinematics import load_rebot_kinematics

from ..teleoperator import Teleoperator
from .config_joycon import JoyconTeleopConfig
from .example6_runtime import Example6Runtime

logger = logging.getLogger(__name__)


class JoyconTeleop(Teleoperator):
    config_class = JoyconTeleopConfig
    name = "joycon"

    def __init__(self, config: JoyconTeleopConfig):
        super().__init__(config)
        self.config = config
        self.controller: Any | None = None
        self._runtime = None
        self._example = None
        self._feedback: np.ndarray | None = None
        self._last_button = 0
        self._episode_button = 0
        self._gripper_goal: float | None = None
        self._last_time: float | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return self.action_features

    @property
    def is_connected(self) -> bool:
        return self.controller is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        if self.config.use_example6:
            if (
                self.config.xyz_scale != 2 / 3
                or self.config.rotation_scale != 1.0
                or self.config.gripper_speed_rad_s is not None
                or self.config.max_joint_step_deg is not None
                or self.config.kinematics_path is not None
            ):
                raise ValueError("Example 6 owns all control parameters; remove legacy teleop overrides")
            self._example_module = load_example6(self.config.examples_path)
            self._example = self._example_module.Example6Controller(self.config.software_calibration)
            self.controller = self._example.joycon
            return
        # Lazy: CLI/config parsing and training need neither HID nor kinematics dependencies.
        try:
            from joyconrobotics import JoyconRobotics
        except ImportError as exc:
            raise ImportError(
                "Install the bundled joycon-robotics package; see docs/REBOT_JOYCON.md"
            ) from exc

        self.kinematics = load_rebot_kinematics(self.config.kinematics_path)
        path = self.config.software_calibration
        self.arm_calibration = self.kinematics.ReBotCalibration(path)
        self.model = self.kinematics.create_rebot_b601_dm(calibration=self.arm_calibration)
        self._limits = self.arm_calibration.control_limits(self.model.qlim)
        self._feedback = None
        self._last_button = self._episode_button = 0
        self._gripper_goal = self._last_time = None
        home = self.kinematics.get_home_q(self.model)
        home_pose = self.kinematics.rebot_FK(home, robot=self.model)
        self._last_valid_q = home.copy()
        self.controller = JoyconRobotics(
            device="right",
            gripper_open=self.arm_calibration.gripper_open,
            gripper_close=self.arm_calibration.gripper_close,
            gripper_state=self.arm_calibration.gripper_close,
            offset_position_m=home_pose[:3].tolist(),
            glimit=[[0.0, -0.45, 0.0, -3.1, -3.1, -3.1], [0.45, 0.45, 0.68, 3.1, 3.1, 3.1]],
            dof_speed=[self.config.xyz_scale] * 3 + [self.config.rotation_scale] * 3,
            pitch_down_double=False,
        )

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        values = np.array([feedback[f"{name}.pos"] for name in JOINT_NAMES], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("Non-finite reBot feedback")
        self._feedback = np.deg2rad(values)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        if self.config.use_example6:
            if self._runtime is None:
                raise DeviceNotConnectedError("Example 6 control worker is not attached to the robot")
            return self._runtime.snapshot()[1]
        if self._feedback is None:
            raise DeviceNotConnectedError("Joy-Con requires robot feedback before its first action")
        pose, gripper, button = self.controller.get_control()
        # Rising edges prevent a held button from skipping multiple episodes.
        if button in (1, -1) and button != self._last_button:
            self._episode_button = button
        self._last_button = button
        pose = np.asarray(pose, dtype=float).copy()
        if pose.shape != (6,) or not np.isfinite(pose).all() or not np.isfinite(gripper):
            raise ValueError("Invalid Joy-Con pose or gripper input")
        now = time.monotonic()
        dt = 0.0 if self._last_time is None else min(max(now - self._last_time, 0.0), 0.1)
        self._last_time = now
        if self._gripper_goal is None:
            self._gripper_goal = self.arm_calibration.gripper_close
            self._gripper_target = self._feedback[6]
        # Absolute Cartesian target and roll/pitch signs match example 6.
        target = pose.copy()
        target[3:5] *= -1
        q, success = self.kinematics.rebot_IK(self._last_valid_q, target, robot=self.model, quiet=True)
        q = np.asarray(q)
        if not success or q.shape != (6,) or not np.isfinite(q).all():
            q = self._last_valid_q.copy()
            gripper = self._gripper_goal
        if self.config.max_joint_step_deg is not None:
            max_step = np.deg2rad(self.config.max_joint_step_deg)
            q = np.clip(q, self._feedback[:6] - max_step, self._feedback[:6] + max_step)
        q = np.clip(q, self._limits[0, :6], self._limits[1, :6])
        self._last_valid_q = q.copy()
        self._gripper_goal = float(gripper)
        if self.config.gripper_speed_rad_s is None:
            self._gripper_target = self._gripper_goal
        else:
            step = self.config.gripper_speed_rad_s * dt
            self._gripper_target += np.clip(self._gripper_goal - self._gripper_target, -step, step)
        self._gripper_target = float(np.clip(self._gripper_target, *self._limits[:, 6]))
        return dict(
            zip(self.action_features, np.rad2deg(np.r_[q, self._gripper_target]).tolist(), strict=True)
        )

    def start_control(self, robot, *, background: bool = True) -> None:
        if not self.config.use_example6:
            return
        if robot._example_sdk is None:
            raise ValueError("Example 6 Joy-Con requires the examples robot backend")
        self._runtime = Example6Runtime(self._example_module, self._example, robot)
        robot._joycon_runtime = self._runtime
        self._runtime.start(background=background)

    def run_foreground(self, duration_s: float | None, on_step: Callable[[], None] | None = None) -> None:
        if self._runtime is None:
            raise DeviceNotConnectedError("Example 6 control is not attached to the robot")
        self._runtime.run_foreground(duration_s, on_step)

    def run_recording(self, record_session: Callable[[], None], cancel: Callable[[], None]) -> None:
        if self._runtime is None:
            raise DeviceNotConnectedError("Example 6 control is not attached to the robot")
        self._runtime.run_recording(record_session, cancel)

    def stop_control(self, *, check_error: bool = False) -> None:
        if self._runtime is not None:
            runtime = self._runtime
            runtime.stop()
            runtime.robot._joycon_runtime = None
            self._runtime = None
            if check_error:
                runtime.snapshot()  # Do not start a home motion after a late worker fault.

    def update_recording_events(self, events: dict[str, bool]) -> None:
        """Consume A/Y once; Y only discards the pending, unsaved episode."""
        if self._runtime is not None:
            buttons = self._runtime.consume_buttons()
            if buttons:
                events["exit_early"] = True
                if -1 in buttons:
                    events["rerecord_episode"] = True
            return
        button, self._episode_button = self._episode_button, 0
        if button == 1:
            events["exit_early"] = True
        elif button == -1:
            events["rerecord_episode"] = True
            events["exit_early"] = True

    @check_if_not_connected
    def disconnect(self) -> None:
        self.stop_control()
        controller, self.controller = self.controller, None
        controller.running = False
        controller.thread.join(timeout=2.0)
        # The bundled joyconrobotics owns three HID readers; its API has no public close().
        for name in ("joycon", "gyro", "button"):
            reader = getattr(controller, name, None)
            if reader is not None:
                reader.enable = False
                try:
                    reader._close()
                except Exception:
                    logger.exception("Failed to close Joy-Con %s reader", name)
        self._feedback = None
        self._last_button = self._episode_button = 0


def validate_joycon_robot(teleop: Teleoperator | None, robot: Any) -> None:
    """Refuse an incompatible coordinate frame before either device is connected."""
    if not isinstance(teleop, JoyconTeleop):
        return
    if robot.name != "rebot_b601_follower" or robot.config.motor_family.value != "dm":
        raise ValueError("This Joy-Con adapter requires rebot_b601_follower with motor_family=dm")
    if teleop.config.use_example6:
        if robot.config.control_backend != "examples":
            raise ValueError("Example 6 Joy-Con requires robot.control_backend=examples")
        if example_directory(teleop.config.examples_path) != example_directory(robot.config.examples_path):
            raise ValueError("Robot and Joy-Con must share examples_path")
        if (
            teleop.config.software_calibration is None
            or robot.config.software_calibration is None
            or teleop.config.software_calibration.resolve() != robot.config.software_calibration.resolve()
        ):
            raise ValueError("Joy-Con and robot must use the same software_calibration JSON file")
        robot._defer_example_home = True
        return
    if robot.config.control_backend == "examples":
        raise ValueError("Legacy Joy-Con requires robot.control_backend=motorbridge")
    kin = load_rebot_kinematics(teleop.config.kinematics_path)
    robot_kin = load_rebot_kinematics(robot.config.kinematics_path)
    if kin.__file__ != robot_kin.__file__:
        raise ValueError("Joy-Con and robot must share the same kinematics file")
    path = teleop.config.software_calibration
    calibration = kin.ReBotCalibration(path)
    if robot._software_calibration is None or calibration.path != robot._software_calibration.path:
        raise ValueError("Joy-Con and robot must use the same software_calibration JSON file")
