"""Real example FK/IK and LeRobot contracts, fake HID/CAN; never moves hardware."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import draccus
import numpy as np
import pytest

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig
from lerobot.robots.rebot_b601_follower.motor_family import JOINT_NAMES
from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config
from lerobot.teleoperators.joycon import JoyconTeleop, JoyconTeleopConfig
from lerobot.teleoperators.joycon.teleop_joycon import validate_joycon_robot
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.rebot_kinematics import load_rebot_kinematics

CALIBRATION_PATH = Path(__file__).resolve().parents[4] / "examples/calibration/rebot.json"

MODULE = "lerobot.teleoperators.joycon.teleop_joycon"
ROBOT_MODULE = "lerobot.robots.rebot_b601_follower.rebot_b601_follower"


@pytest.fixture
def connected(tmp_path):
    kin = load_rebot_kinematics()
    calibration = kin.ReBotCalibration(CALIBRATION_PATH)
    controller = MagicMock()
    controller.get_control.return_value = ([0.0] * 6, 0.0, 0)
    factory = MagicMock(return_value=controller)
    teleop = JoyconTeleop(
        JoyconTeleopConfig(
            use_example6=False,
            calibration_dir=tmp_path,
            gripper_speed_rad_s=0.5,
            software_calibration=CALIBRATION_PATH,
        )
    )
    with patch.dict(sys.modules, joyconrobotics=SimpleNamespace(JoyconRobotics=factory)):
        teleop.connect()
    q = kin.get_home_q(teleop.model)
    pose = kin.rebot_FK(q, robot=teleop.model)
    pose[3:5] *= -1
    controller.get_control.return_value = (pose, calibration.gripper_close, 0)
    feedback = dict(zip(teleop.action_features, np.rad2deg(np.r_[q, calibration.gripper_close]), strict=True))
    try:
        yield teleop, controller, feedback, factory
    finally:
        if teleop.is_connected:
            teleop.disconnect()


def test_registry_without_hardware(tmp_path):
    config = draccus.decode(TeleoperatorConfig, {"type": "joycon", "calibration_dir": str(tmp_path)})
    teleop = make_teleoperator_from_config(config)
    assert isinstance(teleop, JoyconTeleop)
    assert not teleop.is_connected
    assert tuple(teleop.action_features) == tuple(f"{name}.pos" for name in JOINT_NAMES)
    with pytest.raises(DeviceNotConnectedError):
        teleop.get_action()


def test_home_target_and_movement_use_shared_ik(connected):
    teleop, controller, feedback, factory = connected
    factory.assert_called_once()
    assert factory.call_args.kwargs["dof_speed"] == [2 / 3] * 3 + [1.0] * 3
    with pytest.raises(DeviceNotConnectedError):
        teleop.get_action()
    teleop.send_feedback(feedback)
    first = teleop.get_action()
    # Shared example IK regularizes near-zero Euler angles; compare TCP, not identical joints.
    initial_tcp = teleop.kinematics.rebot_FK(np.deg2rad(list(first.values())[:6]), robot=teleop.model)
    assert initial_tcp[2] == pytest.approx(0.25, abs=2e-5)
    pose = controller.get_control.return_value[0].copy()
    pose[2] += 0.002
    controller.get_control.return_value = (pose, teleop.arm_calibration.gripper_close, 0)
    moved = teleop.get_action()
    q = np.deg2rad(list(moved.values())[:6])
    tcp = teleop.kinematics.rebot_FK(q, robot=teleop.model)
    assert tcp[2] == pytest.approx(0.252, abs=2e-5)
    assert moved["gripper.pos"] == first["gripper.pos"]


def test_ik_failure_holds_last_valid_arm_and_gripper(connected):
    teleop, controller, feedback, _ = connected
    teleop.send_feedback(feedback)
    with patch(f"{MODULE}.time.monotonic", side_effect=[1.0, 1.1, 2.0]):
        first = teleop.get_action()
        controller.get_control.return_value = ([9, 9, 9, 0, 0, 0], 1.0, 0)
        with patch.object(teleop.kinematics, "rebot_IK", return_value=(np.zeros(6), False)):
            moved = teleop.get_action()
            stalled = teleop.get_action()
    np.testing.assert_allclose(list(moved.values())[:6], list(first.values())[:6], atol=1e-12)
    assert moved["gripper.pos"] == first["gripper.pos"]
    assert stalled["gripper.pos"] == first["gripper.pos"]


def test_limits_nan_and_disconnect(connected):
    teleop, controller, feedback, _ = connected
    teleop.send_feedback(feedback)
    teleop.get_action()
    teleop.config.max_joint_step_deg = 5.0
    with patch.object(teleop.kinematics, "rebot_IK", return_value=(np.full(6, 100.0), True)):
        action = teleop.get_action()
    assert max(abs(action[key] - feedback[key]) for key in list(action)[:6]) <= 5.0 + 1e-8
    feedback["shoulder_pan.pos"] = float("nan")
    with pytest.raises(ValueError):
        teleop.send_feedback(feedback)
    controller.get_control.return_value = ([float("nan")] * 6, 0, 0)
    with pytest.raises(ValueError):
        teleop.get_action()
    teleop.disconnect()
    assert not teleop.is_connected
    assert controller.running is False
    controller.thread.join.assert_called_once()
    for name in ("joycon", "gyro", "button"):
        getattr(controller, name)._close.assert_called_once()


def test_robot_calibration_roundtrip_and_hardware_zero_not_written(tmp_path):
    kin = load_rebot_kinematics()
    cal = kin.ReBotCalibration(CALIBRATION_PATH)
    config = RebotB601FollowerRobotConfig(
        control_backend="motorbridge",
        port="/dev/null",
        software_calibration=cal.path,
        calibration_dir=tmp_path,
    )
    with patch(f"{ROBOT_MODULE}.require_package"):
        robot = RebotB601Follower(config)
    assert robot.is_calibrated
    robot.motors = {name: MagicMock() for name in JOINT_NAMES}
    robot.calibrate()
    for i, name in enumerate(JOINT_NAMES):
        raw = robot._public_to_motor_position(name, 10.0)
        assert robot._motor_to_public_position(name, raw) == pytest.approx(10.0)
        assert raw == pytest.approx(10.0 * cal.signs[i] + np.rad2deg(cal.offsets[i]))
        robot.motors[name].set_zero_position.assert_not_called()
    # The policy uses the same calibrated joint coordinates as recording.
    raw_states = {name: SimpleNamespace(pos=float(cal.offsets[i])) for i, name in enumerate(JOINT_NAMES)}
    np.testing.assert_allclose(list(robot._present_pos(raw_states).values()), 0.0, atol=1e-12)
    teleop = JoyconTeleop(
        JoyconTeleopConfig(
            use_example6=False, calibration_dir=tmp_path / "teleop", software_calibration=CALIBRATION_PATH
        )
    )
    validate_joycon_robot(teleop, robot)
    robot._software_calibration = None
    with pytest.raises(ValueError, match="same software_calibration"):
        validate_joycon_robot(teleop, robot)


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_invalid_speed(value):
    with pytest.raises(ValueError):
        JoyconTeleopConfig(use_example6=False, gripper_speed_rad_s=value)


def test_recorded_actions_roundtrip_into_act_training(connected, tmp_path):
    import torch

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_record import record_loop
    from lerobot.utils.feature_utils import dataset_to_policy_features, hw_to_dataset_features

    teleop, _, feedback, _ = connected
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    features = {
        **hw_to_dataset_features(teleop.action_features, "action", use_video=False),
        **hw_to_dataset_features(
            {**teleop.action_features, "front": image.shape}, "observation", use_video=False
        ),
    }
    dataset_root = tmp_path / "dataset"
    dataset = LeRobotDataset.create(
        "local/rebot_offline",
        fps=30,
        features=features,
        root=dataset_root,
        robot_type="rebot_b601_follower",
        use_videos=False,
    )
    events = {"exit_early": False}
    sent = []

    def send(action):
        # Simulate the Robot's final clamp; recording must save this value, not the requested one.
        result = dict(action)
        result["shoulder_pan.pos"] += 0.01
        sent.append(result)
        if len(sent) == 4:
            events["exit_early"] = True
        return result

    robot = SimpleNamespace(
        name="rebot_b601_follower", get_observation=lambda: {**feedback, "front": image}, send_action=send
    )
    processors = make_default_processors()
    record_loop(
        robot=robot,
        events=events,
        fps=30,
        teleop_action_processor=processors[0],
        robot_action_processor=processors[1],
        robot_observation_processor=processors[2],
        teleop=teleop,
        dataset=dataset,
        control_time_s=5,
        single_task="offline fixture",
    )
    assert len(sent) == 4
    dataset.save_episode()
    dataset.finalize()
    loaded = LeRobotDataset("local/rebot_offline", root=dataset_root, video_backend="pyav")
    sample = loaded[0]
    np.testing.assert_allclose(sample["action"], list(sent[0].values()), atol=1e-5)
    assert sample["observation.state"].shape == (7,)
    assert sample["observation.images.front"].shape == (3, 64, 64)
    policy_features = dataset_to_policy_features(loaded.features)
    cfg = ACTConfig(
        input_features={k: v for k, v in policy_features.items() if k != "action"},
        output_features={"action": policy_features["action"]},
        pretrained_backbone_weights=None,
        device="cpu",
        chunk_size=2,
        n_action_steps=2,
        dim_model=32,
        n_heads=4,
        dim_feedforward=64,
        n_encoder_layers=1,
        n_decoder_layers=1,
        n_vae_encoder_layers=1,
    )
    old_threads = torch.get_num_threads()
    torch.set_num_threads(1)
    try:
        policy = ACTPolicy(cfg)
        batch = {
            "observation.state": sample["observation.state"].unsqueeze(0),
            "observation.images.front": sample["observation.images.front"].unsqueeze(0),
            "action": sample["action"].reshape(1, 1, 7).repeat(1, 2, 1),
            "action_is_pad": torch.zeros((1, 2), dtype=torch.bool),
        }
        loss, _ = policy(batch)
        assert torch.isfinite(loss)
        loss.backward()
        assert any(p.grad is not None for p in policy.parameters())
    finally:
        torch.set_num_threads(old_threads)


def test_bundled_model_matches_original_and_needs_no_external_code():
    bundled = load_rebot_kinematics()
    assert Path(bundled.__file__).parent.name == "joycon"
    # Constructing the pure kinematic model does not read calibration or examples files.
    with patch.object(Path, "read_text", side_effect=AssertionError("No external files")):
        pure_model = bundled.get_robot()
        assert bundled.rebot_FK(bundled.get_home_q(pure_model), robot=pure_model)[2] == pytest.approx(
            0.25, abs=1e-6
        )
    original_path = CALIBRATION_PATH.parents[1] / "ReBot_Kinematics.py"
    original = load_rebot_kinematics(original_path)
    calibration = bundled.ReBotCalibration(CALIBRATION_PATH)
    copied_model = bundled.create_rebot_b601_dm(calibration=calibration)
    original_model = original.create_rebot_b601_dm(calibration=original.ReBotCalibration(CALIBRATION_PATH))
    for field in ("REBOT_DH", "REBOT_BASE_TRANSFORM", "REBOT_END_TRANSFORM", "REBOT_TCP_OFFSET"):
        np.testing.assert_array_equal(getattr(bundled, field), getattr(original, field))
    rng = np.random.default_rng(42)
    for q in rng.uniform(copied_model.qlim[0], copied_model.qlim[1], size=(100, 6)):
        np.testing.assert_allclose(copied_model.fkine(q).A, original_model.fkine(q).A, atol=1e-12)
    np.testing.assert_allclose(
        bundled.get_home_q(copied_model), original.get_home_q(original_model), atol=1e-12
    )
    assert load_rebot_kinematics() is bundled
    assert load_rebot_kinematics(Path(bundled.__file__)) is bundled


def test_bundled_calibration_requires_explicit_data_path():
    with pytest.raises(ValueError, match="software_calibration"):
        load_rebot_kinematics().ReBotCalibration()


def test_example_controller_defaults_and_direct_gripper(connected):
    teleop, controller, feedback, factory = connected
    kwargs = factory.call_args.kwargs
    assert kwargs["pitch_down_double"] is False
    assert kwargs["glimit"] == [[0.0, -0.45, 0.0, -3.1, -3.1, -3.1], [0.45, 0.45, 0.68, 3.1, 3.1, 3.1]]
    assert kwargs["offset_position_m"][2] == pytest.approx(0.25, abs=1e-6)
    assert kwargs["gripper_open"] == teleop.arm_calibration.gripper_open
    teleop.config.gripper_speed_rad_s = None
    teleop.send_feedback(feedback)
    pose = controller.get_control.return_value[0]
    controller.get_control.return_value = (pose, teleop.arm_calibration.gripper_open, 0)
    action = teleop.get_action()
    assert action["gripper.pos"] == pytest.approx(np.rad2deg(teleop.arm_calibration.gripper_open))


def test_recording_button_edges(connected):
    teleop, controller, feedback, _ = connected
    teleop.send_feedback(feedback)
    pose, gripper, _ = controller.get_control.return_value
    events = {"exit_early": False, "rerecord_episode": False}
    for button, expected_exit, expected_redo in [
        (1, True, False),
        (1, False, False),
        (0, False, False),
        (-1, True, True),
        (-1, False, False),
    ]:
        controller.get_control.return_value = (pose, gripper, button)
        teleop.get_action()
        teleop.update_recording_events(events)
        assert events == {"exit_early": expected_exit, "rerecord_episode": expected_redo}
        events = {"exit_early": False, "rerecord_episode": False}


@pytest.mark.parametrize(
    "gripper_error,gripper_velocity,expected_gripper_torque",
    [(0.0, 0.0, 0.0), (0.05, 0.2, 0.76), (0.05, -30.0, 3.0)],
    ids=["at-home", "reduced-gain-and-damping", "torque-cap"],
)
def test_calibrated_robot_home_gains_and_timeout(
    tmp_path: Path, gripper_error: float, gripper_velocity: float, expected_gripper_torque: float
) -> None:
    config = RebotB601FollowerRobotConfig(
        control_backend="motorbridge",
        port="/dev/null",
        software_calibration=CALIBRATION_PATH,
        calibration_dir=tmp_path,
    )
    with patch(f"{ROBOT_MODULE}.require_package"):
        robot = RebotB601Follower(config)
    assert config.gripper_control_mode.value == "mit"
    assert list(config.mit_kp.values()) == [100.0, 100.0, 100.0, 24.0, 24.0, 24.0, 24.0]
    assert list(config.mit_kd.values()) == [5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 1.0]
    robot.bus = MagicMock()
    robot.motors = {name: MagicMock() for name in JOINT_NAMES}
    home = {key.removesuffix(".pos"): val for key, val in robot._home_action.items()}
    states = {
        name: SimpleNamespace(pos=np.deg2rad(robot._public_to_motor_position(name, val)), vel=0.0, torq=0.0)
        for name, val in home.items()
    }
    # MotorBridge feedback includes position, velocity, and measured torque.
    # Keep the gripper within home tolerance while exercising its host-side PD.
    states["gripper"].pos -= gripper_error
    states["gripper"].vel = gripper_velocity
    with patch.object(robot, "_read_feedback", return_value=states):
        robot.go_home()
    for name, motor in robot.motors.items():
        args = motor.send_mit.call_args.args
        if name == "gripper":
            # MotorBridge caps the combined PD torque and sends zero motor-side gains.
            assert args == pytest.approx((0.0, 0.0, 0.0, 0.0, expected_gripper_torque))
        else:
            assert args[2] == pytest.approx(config.mit_kp[name] * config.home_kp_scale)
            assert args[3] == config.mit_kd[name]
    robot.bus.disable_all.assert_not_called()
    assert not robot._homing
    with patch(f"{ROBOT_MODULE}.time.monotonic", side_effect=[0.0, 16.0]), pytest.raises(TimeoutError):
        robot.go_home()
    robot.bus.disable_all.assert_called_once()
    assert not robot._homing
    for motor in robot.motors.values():
        motor.get_state.return_value = SimpleNamespace(pos=float("nan"))
    with pytest.raises(RuntimeError, match="Non-finite"):
        robot._read_feedback()


@pytest.mark.parametrize("button,redo", [(1, False), (-1, True)])
def test_record_loop_consumes_joycon_buttons(connected, button, redo):
    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_record import record_loop

    teleop, controller, feedback, _ = connected
    pose, gripper, _ = controller.get_control.return_value
    controller.get_control.return_value = (pose, gripper, button)
    events = {"exit_early": False, "rerecord_episode": False}
    send = MagicMock(side_effect=lambda action: action)
    robot = SimpleNamespace(name="rebot_b601_follower", get_observation=lambda: feedback, send_action=send)
    processors = make_default_processors()
    record_loop(
        robot=robot,
        events=events,
        fps=30,
        teleop_action_processor=processors[0],
        robot_action_processor=processors[1],
        robot_observation_processor=processors[2],
        teleop=teleop,
        control_time_s=1.0,
    )
    send.assert_called_once()
    assert events == {"exit_early": False, "rerecord_episode": redo}


@pytest.mark.parametrize("fail", [False, True])
def test_calibrated_connect_homes_and_cleans_up_on_failure(tmp_path, fail):
    config = RebotB601FollowerRobotConfig(
        control_backend="motorbridge",
        port="/dev/null",
        software_calibration=CALIBRATION_PATH,
        calibration_dir=tmp_path,
    )
    with (
        patch(f"{ROBOT_MODULE}.require_package"),
        patch(f"{ROBOT_MODULE}.MotorBridgeController") as controller,
        patch(f"{ROBOT_MODULE}.MotorBridgeMode"),
    ):
        robot = RebotB601Follower(config)
        bus = controller.from_dm_serial.return_value
        with patch.object(
            robot, "go_home", side_effect=TimeoutError("test timeout") if fail else None
        ) as home:
            if fail:
                with pytest.raises(TimeoutError):
                    robot.connect()
                assert not robot.is_connected
                bus.disable_all.assert_called()
                bus.close.assert_called_once()
            else:
                robot.connect()
                assert robot.is_connected
                robot.disconnect()
            home.assert_called_once()


def test_calibrated_config_roundtrip_keeps_example_defaults(tmp_path):
    config = RebotB601FollowerRobotConfig(
        control_backend="motorbridge",
        port="/dev/null",
        software_calibration=CALIBRATION_PATH,
        calibration_dir=tmp_path,
        mit_kp={"gripper": 12.0},
    )
    decoded = draccus.decode(RebotB601FollowerRobotConfig, draccus.encode(config))
    assert decoded.mit_kp["shoulder_pan"] == 100.0
    assert decoded.mit_kp["gripper"] == 12.0
    with patch(f"{ROBOT_MODULE}.require_package"):
        original = RebotB601Follower(config)
        restored = RebotB601Follower(decoded)
    assert restored._joint_limits == original._joint_limits
    cal = load_rebot_kinematics().ReBotCalibration(CALIBRATION_PATH)
    expected = cal.control_limits(load_rebot_kinematics().REBOT_JOINT_LIMITS.T)
    np.testing.assert_allclose(list(restored._joint_limits.values()), np.rad2deg(expected.T))
