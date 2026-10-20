"""Compare the shared controller against the pre-refactor example loop; fake HID/CAN only."""

import sys
import textwrap
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from lerobot.robots.rebot_b601_follower import RebotB601Follower, RebotB601FollowerRobotConfig
from lerobot.teleoperators.joycon import JoyconTeleop, JoyconTeleopConfig
from lerobot.teleoperators.joycon.example6_runtime import Example6Runtime
from lerobot.teleoperators.joycon.teleop_joycon import validate_joycon_robot
from lerobot.utils.rebot_example6 import load_example6

CALIBRATION = Path(__file__).resolve().parents[4] / "examples/calibration/rebot.json"


class FakeArm:
    def __init__(self, home):
        self.home_pose = np.array(home)
        self.target_pose = self.home_pose.copy()
        self.motors_pose = self.home_pose.copy()
        self.commands = []
        self.disabled = False

    def Go_Home_Pose(self):  # noqa: N802 - original SDK API
        self.target_pose = self.home_pose.copy()
        self.commands.append(("home", self.target_pose.copy()))

    def ReBot_Control(self, q, *, smooth=False):  # noqa: N802 - original SDK API
        self.target_pose = np.array(q).copy()
        self.commands.append(("control", self.target_pose.copy()))

    def ReBot_Disable(self):  # noqa: N802 - original SDK API
        self.disabled = True


@pytest.fixture
def example(monkeypatch):
    module = load_example6()
    factory = MagicMock(side_effect=lambda **kwargs: MagicMock())
    # Restore only this fake HID module. Restoring all of sys.modules after a
    # recording test unloads pandas/policy modules while their registries survive.
    monkeypatch.setitem(sys.modules, "joyconrobotics", SimpleNamespace(JoyconRobotics=factory))
    controller = module.Example6Controller(CALIBRATION)
    yield module, controller, factory
    controller.close_joycon()


def test_shared_controller_matches_original_loop(example):
    module, current, factory = example
    original = (Path(__file__).parents[1] / "fixtures/rebot_example6_original.py.txt").read_text()
    scope = {"__file__": module.__file__}
    with patch.dict(sys.modules, joyconrobotics=SimpleNamespace(JoyconRobotics=factory)):
        exec(compile(original[: original.index("# auto ckeck")], "original-example-init", "exec"), scope)
    old_args, new_args = factory.call_args_list[1].kwargs, factory.call_args_list[0].kwargs
    assert old_args.keys() == new_args.keys()
    for key in old_args:
        if isinstance(old_args[key], str):
            assert old_args[key] == new_args[key]
        else:
            np.testing.assert_allclose(old_args[key], new_args[key], atol=1e-12)
    old_arm, new_arm = FakeArm(scope["init_qpos"]), FakeArm(current.init_qpos)
    scope.update(rebot_right=old_arm, viewer=MagicMock(), t=0)
    old_arm.Go_Home_Pose()
    current.attach(new_arm)
    loop = textwrap.dedent(
        original[original.index("            if t ==0 :") : original.index("            # Time management")]
    )
    compiled = compile(loop, "original-example-loop", "exec")
    for i in range(45):
        pose = current.init_gpos.copy()
        pose[2] += 0.002 * min(i, 5) / 5
        # Arm behavior still matches the original; gripper ramp is tested separately.
        gripper = current.calibration.gripper_close
        scope["joyconrobotics_right"].get_control.return_value = (pose.copy(), gripper, 0)
        current.joycon.get_control.return_value = (pose.copy(), gripper, 0)
        exec(compiled, scope)
        current.step()
        np.testing.assert_allclose(current.mjdata.qpos, scope["mjdata"].qpos, atol=1e-10)
        np.testing.assert_allclose(current.target_qpos, scope["target_qpos"], atol=1e-10)
        assert len(old_arm.commands) == len(new_arm.commands)
        assert old_arm.commands[-1][0] == new_arm.commands[-1][0]
        np.testing.assert_allclose(old_arm.commands[-1][1], new_arm.commands[-1][1], atol=1e-10)
    assert [kind for kind, _ in new_arm.commands] == ["home"] * 40 + ["control"] * 5
    # Failed IK holds the six axes while continuing gripper/feedback control.
    before = new_arm.target_pose.copy()
    with patch.object(module, "rebot_IK", return_value=(np.zeros(6), False)):
        current.step()
    assert len(new_arm.commands) == 46
    np.testing.assert_allclose(new_arm.target_pose[:6], before[:6])


def test_gripper_timed_ramp_reversal_limits_and_failed_ik(example):
    module, controller, _ = example
    arm = FakeArm(controller.init_qpos)
    with patch.object(module.time, "monotonic", return_value=0.0):
        controller.attach(arm)
    controller.t = module.STARTUP_HOME_CYCLES
    closed = controller.calibration.gripper_close
    opened = controller.calibration.gripper_open
    controller.joycon.get_control.return_value = (controller.init_gpos.copy(), opened - 10, 0)
    with patch.object(module, "rebot_IK", return_value=(controller.init_qpos[:6], True)):
        for i in range(1, 11):
            with patch.object(module.time, "monotonic", return_value=i * 0.1):
                controller.step()
            assert arm.target_pose[6] == pytest.approx(closed + (opened - closed) * i / 10)
        controller.joycon.get_control.return_value = (controller.init_gpos.copy(), closed + 10, 0)
        with patch.object(module.time, "monotonic", return_value=1.1):
            controller.step()
        assert arm.target_pose[6] == pytest.approx(opened + (closed - opened) / 10)
    before = arm.target_pose.copy()
    with (
        patch.object(module, "rebot_IK", return_value=(np.zeros(6), False)),
        patch.object(module.time, "monotonic", return_value=1.2),
    ):
        controller.step()
    np.testing.assert_array_equal(arm.target_pose[:6], before[:6])
    assert arm.target_pose[6] == pytest.approx(before[6] + (closed - opened) / 10)
    before = arm.target_pose.copy()
    with (
        patch.object(module, "rebot_IK", return_value=(controller.init_qpos[:6], True)),
        patch.object(module.time, "monotonic", return_value=10.0),
    ):
        controller.step()
    assert arm.target_pose[6] == pytest.approx(before[6] + (closed - opened) / 10)


def test_gripper_finishes_both_directions_during_continuous_failed_ik(example):
    module, controller, _ = example
    arm = FakeArm(controller.init_qpos)
    with patch.object(module.time, "monotonic", return_value=0.):
        controller.attach(arm)
    controller.t = module.STARTUP_HOME_CYCLES
    held = arm.target_pose[:6].copy()
    for direction, endpoint in enumerate((controller.calibration.gripper_open, controller.calibration.gripper_close)):
        controller.joycon.get_control.return_value = (controller.init_gpos.copy(), endpoint, 0)
        before = len(arm.commands)
        with patch.object(module, "rebot_IK", return_value=(np.full(6, np.nan), False)):
            for tick in range(1, 101):
                with patch.object(module.time, "monotonic", return_value=direction + tick * .01):
                    controller.step()
                np.testing.assert_array_equal(arm.target_pose[:6], held)
                assert np.isfinite(arm.target_pose).all()
        assert len(arm.commands) - before == 100
        assert arm.target_pose[6] == pytest.approx(endpoint)


def test_default_uses_original_sdk_and_sampling_does_not_resend(example, tmp_path):
    module, controller, _ = example
    config = RebotB601FollowerRobotConfig(
        port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path
    )
    robot = RebotB601Follower(config)
    assert config.control_backend == "examples"
    assert module.ReBot.__module__ == "ReBot_SDK"
    sdk = FakeArm(controller.init_qpos)
    sdk.MotorController = SimpleNamespace(serial_=MagicMock())
    sdk.update_motors_status = MagicMock()
    with patch.object(module, "ReBot", return_value=sdk) as factory:
        robot.connect()
    factory.assert_called_once_with(driver_port="/dev/null", calibration_path=CALIBRATION)
    action = {
        f"{name}.pos": float(value)
        for name, value in zip(robot.motor_names, np.rad2deg(controller.init_qpos), strict=True)
    }
    assert robot.send_action(action) == pytest.approx(action)
    before = len(sdk.commands)
    runtime = SimpleNamespace(snapshot=lambda: (action, action), stop=MagicMock())
    robot._joycon_runtime = runtime
    for _ in range(5):
        assert robot.send_action(action) == action
        assert robot.get_observation() == action
    assert len(sdk.commands) == before
    robot.disconnect()
    assert sdk.disabled
    runtime.stop.assert_called_once()
    sdk.MotorController.serial_.close.assert_called_once()


def test_worker_runs_independently_and_latches_short_button_presses(example):
    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    robot = SimpleNamespace(_example_lock=threading.RLock(), _example_sdk=sdk)
    runtime = Example6Runtime(module, controller, robot)
    counter = 0

    def step():
        nonlocal counter
        counter += 1
        if counter == 5:
            runtime._stop.set()
        return controller.init_qpos, [1, 1, 0, -1, 0][counter - 1]

    with patch.object(controller, "step", side_effect=step):
        runtime.start()
        runtime.thread.join(timeout=2.0)
    assert not runtime.thread.is_alive()
    assert counter == 5  # No recording/get_action/send_action calls were needed.
    assert runtime.consume_buttons() == [1, -1]
    assert runtime.consume_buttons() == []
    runtime.stop()


def test_worker_fault_disables_arm_and_reaches_caller(example):
    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    robot = SimpleNamespace(_example_lock=threading.RLock(), _example_sdk=sdk)
    runtime = Example6Runtime(module, controller, robot)
    with patch.object(controller, "step", side_effect=ValueError("fixture fault")):
        runtime.start()
        runtime.thread.join(timeout=2.0)
    assert sdk.disabled
    with pytest.raises(RuntimeError, match="loop failed"):
        runtime.snapshot()
    runtime.stop()


def test_foreground_matches_standalone_gripper_trajectory_and_viewer(example):
    import mujoco.viewer

    module, standalone, factory = example
    with patch.dict(sys.modules, joyconrobotics=SimpleNamespace(JoyconRobotics=factory)):
        foreground = module.Example6Controller(CALIBRATION)
    results = []
    for controller in (standalone, foreground):
        controller.t = module.STARTUP_HOME_CYCLES
        controller.joycon.get_control.return_value = (
            controller.init_gpos.copy(), controller.calibration.gripper_open, 0
        )
        sdk = FakeArm(controller.init_qpos)
        viewer = MagicMock()
        viewer.is_running.side_effect = [True] * 61 + [False]
        clock = [0.0]

        def advance(_seconds):
            clock[0] += 0.02

        with (
            patch.object(mujoco.viewer, "launch_passive") as launch,
            patch.object(module.time, "monotonic", side_effect=lambda: clock[0]),
            patch.object(module.time, "sleep", side_effect=advance),
        ):
            launch.return_value.__enter__.return_value = viewer
            if controller is standalone:
                controller.attach(sdk)
                module.run_control_loop(controller, duration_s=None)
            else:
                robot = SimpleNamespace(_example_lock=threading.RLock(), _example_sdk=sdk)
                runtime = Example6Runtime(module, controller, robot)
                runtime.start(background=False)
                assert runtime.thread.ident is None
                runtime.run_foreground(duration_s=None)
                assert runtime.thread.ident is None
                assert runtime.snapshot()[1]["gripper.pos"] == pytest.approx(
                    np.rad2deg(controller.calibration.gripper_open)
                )
                runtime.stop()
            launch.assert_called_once_with(controller.mjmodel, controller.mjdata)
        assert viewer.sync.call_count == 122  # Same two viewer syncs per successful step.
        results.append(np.array([q for kind, q in sdk.commands if kind == "control"]))
    np.testing.assert_allclose(results[0], results[1], atol=1e-12)
    assert len(results[0]) == 61
    assert results[1][-1, 6] == pytest.approx(foreground.calibration.gripper_open)


def test_foreground_duration_and_overrun_yield(example):
    import mujoco.viewer

    module, controller, _ = example
    viewer = MagicMock()
    # One step takes 20 ms, over the 1 ms budget; it must still call sleep(0).
    with (
        patch.object(mujoco.viewer, "launch_passive") as launch,
        patch.object(module.time, "monotonic", side_effect=[0., 0., 0., .02, .02]),
        patch.object(module.time, "sleep") as sleep,
        patch.object(controller, "step", return_value=(controller.init_qpos, 0)) as step,
    ):
        launch.return_value.__enter__.return_value = viewer
        module.run_control_loop(controller, duration_s=.01)
    step.assert_called_once_with(viewer)
    sleep.assert_called_once_with(0.)


@pytest.mark.parametrize("failure", [ValueError("viewer/control fault"), KeyboardInterrupt()])
def test_foreground_fault_disables_arm(example, failure):
    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    runtime = Example6Runtime(module, controller, SimpleNamespace(
        _example_lock=threading.RLock(), _example_sdk=sdk
    ))
    runtime.start(background=False)
    with (
        patch.object(module, "run_control_loop", side_effect=failure),
        pytest.raises(type(failure)),
    ):
        runtime.run_foreground(60)
    assert sdk.disabled
    assert runtime.thread.ident is None
    with pytest.raises(RuntimeError, match="loop failed"):
        runtime.snapshot()
    runtime.stop()


def test_pure_teleop_uses_foreground_without_resending_or_outer_timer():
    from lerobot.scripts.lerobot_teleoperate import teleop_loop

    teleop = MagicMock(spec=JoyconTeleop)
    teleop.config = SimpleNamespace(use_example6=True)
    robot = MagicMock()
    processor = MagicMock()
    with patch("lerobot.scripts.lerobot_teleoperate.CycleTimer") as timer:
        teleop_loop(teleop, robot, 30, processor, processor, processor, duration=60)
    teleop.run_foreground.assert_called_once_with(60, None)
    timer.assert_not_called()
    robot.send_action.assert_not_called()
    robot.get_observation.assert_not_called()
    processor.assert_not_called()


def test_teleoperate_starts_foreground_and_cleans_up():
    from lerobot.scripts import lerobot_teleoperate as script

    teleop = MagicMock(spec=JoyconTeleop)
    robot = MagicMock()
    cfg = script.TeleoperateConfig(
        teleop=JoyconTeleopConfig(),
        robot=RebotB601FollowerRobotConfig(port="/dev/null"),
        teleop_time_s=60,
    )
    with (
        patch.object(script, "init_logging"),
        patch.object(script, "make_teleoperator_from_config", return_value=teleop),
        patch.object(script, "make_robot_from_config", return_value=robot),
        patch.object(script, "validate_joycon_robot"),
        patch.object(script, "teleop_loop") as loop,
    ):
        script.teleoperate.__wrapped__(cfg)
    teleop.start_control.assert_called_once_with(robot, background=False)
    assert loop.call_args.kwargs["duration"] == 60
    robot.go_home.assert_called_once()
    robot.disconnect.assert_called_once()
    teleop.disconnect.assert_called_once()


def test_recording_keeps_main_thread_control_during_sampling_reset_and_save(example):
    import mujoco.viewer

    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    runtime = Example6Runtime(module, controller, SimpleNamespace(
        _example_lock=threading.RLock(), _example_sdk=sdk
    ))
    runtime.start(background=False)
    main_thread = threading.get_ident()
    progress = threading.Event()
    cancel = threading.Event()
    count, needed = 0, 100000
    phases = []

    def step(viewer):
        nonlocal count
        assert threading.get_ident() == main_thread
        count += 1
        if count >= needed:
            progress.set()
        return controller.init_qpos, 0

    def sample():
        nonlocal needed
        assert threading.get_ident() != main_thread
        for phase in ("camera", "reset", "save"):
            progress.clear()
            needed = count + 3
            assert progress.wait(2), f"Control stalled during {phase}"
            assert len(runtime.snapshot()[1]) == 7
            phases.append(phase)

    with patch.object(mujoco.viewer, "launch_passive"), patch.object(controller, "step", side_effect=step):
        runtime.run_recording(sample, cancel.set)
    assert phases == ["camera", "reset", "save"]
    assert cancel.is_set()
    assert runtime.thread.ident is None
    assert not sdk.disabled
    runtime.stop()


@pytest.mark.parametrize("failure", ["camera", "window", "interrupt"])
def test_recording_failure_stops_sampler_and_disables_before_cleanup(example, failure):
    import mujoco.viewer

    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    runtime = Example6Runtime(module, controller, SimpleNamespace(
        _example_lock=threading.RLock(), _example_sdk=sdk
    ))
    runtime.start(background=False)
    cancelled, finished = threading.Event(), threading.Event()
    sampler_threads = []
    steps = 0

    def sample():
        sampler_threads.append(threading.current_thread())
        try:
            if failure == "camera":
                raise ValueError("camera read failed")
            assert cancelled.wait(2)
            assert sdk.disabled  # Disable before waiting on camera/dataset shutdown.
        finally:
            finished.set()

    def step(viewer):
        nonlocal steps
        steps += 1
        if failure == "interrupt" and steps == 2:
            raise KeyboardInterrupt()
        return controller.init_qpos, 0

    error_type = {"camera": ValueError, "window": InterruptedError, "interrupt": KeyboardInterrupt}[failure]
    with (
        patch.object(mujoco.viewer, "launch_passive") as launch,
        patch.object(controller, "step", side_effect=step),
        pytest.raises(error_type),
    ):
        if failure == "window":
            launch.return_value.__enter__.return_value.is_running.side_effect = [True, False]
        runtime.run_recording(sample, cancelled.set)
    assert sdk.disabled
    assert cancelled.is_set() and finished.is_set()
    assert len(sampler_threads) == 1 and not sampler_threads[0].is_alive()
    runtime.stop()


@pytest.mark.parametrize("video", [False, True])
def test_record_two_camera_episodes_with_example6_main_loop(example, tmp_path, video):
    import mujoco.viewer

    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts import lerobot_record as script

    module, controller, _ = example
    controller.t = module.STARTUP_HOME_CYCLES
    controller.joycon.get_control.return_value = (
        controller.init_gpos.copy(), controller.calibration.gripper_open, 0
    )
    main_thread = threading.get_ident()
    camera_threads, control_threads, save_threads = [], [], []
    camera = MagicMock()
    camera.is_connected = False
    camera.connect.side_effect = lambda: setattr(camera, "is_connected", True)
    camera.disconnect.side_effect = lambda: setattr(camera, "is_connected", False)

    def read_image():
        camera_threads.append(threading.get_ident())
        return np.zeros((64, 64, 3), dtype=np.uint8)

    camera.read_latest.side_effect = read_image
    robot_config = RebotB601FollowerRobotConfig(
        port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path,
        cameras={"front": OpenCVCameraConfig(index_or_path=0, width=64, height=64, fps=30)},
    )
    with patch(
        "lerobot.robots.rebot_b601_follower.rebot_b601_follower.make_cameras_from_configs",
        return_value={"front": camera},
    ):
        robot = RebotB601Follower(robot_config)
    teleop = JoyconTeleop(JoyconTeleopConfig(software_calibration=CALIBRATION, calibration_dir=tmp_path))
    sdk = FakeArm(controller.init_qpos)
    sdk.MotorController = SimpleNamespace(serial_=MagicMock())
    original_control = sdk.ReBot_Control
    original_save = LeRobotDataset.save_episode

    def control(q):
        control_threads.append(threading.get_ident())
        original_control(q)

    def save(dataset, *args, **kwargs):
        save_threads.append(threading.get_ident())
        before = len(control_threads)
        time.sleep(.03)  # Saving must not suspend the actual control loop.
        assert len(control_threads) > before
        return original_save(dataset, *args, **kwargs)

    sdk.ReBot_Control = control
    cfg = script.RecordConfig(
        robot=robot_config, teleop=teleop.config, play_sounds=False,
        dataset=DatasetRecordConfig(
            repo_id="local/example6_recording", root=tmp_path / "dataset", single_task="open gripper",
            fps=30, num_episodes=2, episode_time_s=.7, reset_time_s=.1,
            video=video, push_to_hub=False, no_stamp=True, encoder_threads=1,
        ),
    )
    with (
        patch.object(module, "Example6Controller", return_value=controller),
        patch.object(module, "ReBot", return_value=sdk),
        patch.object(mujoco.viewer, "launch_passive") as launch,
        patch.object(script, "make_robot_from_config", return_value=robot),
        patch.object(script, "make_teleoperator_from_config", return_value=teleop),
        patch.object(script, "init_keyboard_listener", return_value=(MagicMock(), {
            "exit_early": False, "stop_recording": False, "rerecord_episode": False,
        })),
        patch.object(script, "log_say"),
        patch.object(LeRobotDataset, "save_episode", new=save),
    ):
        dataset = script.record.__wrapped__(cfg)
    launch.assert_called_once()
    assert dataset.num_episodes == 2
    assert camera_threads and all(t != main_thread for t in camera_threads)
    assert len(save_threads) == 2 and all(t != main_thread for t in save_threads)
    assert control_threads and all(t == main_thread for t in control_threads)
    assert len(control_threads) == controller.t - module.STARTUP_HOME_CYCLES
    assert sdk.disabled and not robot.is_connected and not teleop.is_connected
    commands = [q for kind, q in sdk.commands if kind == "control"]
    assert commands[-1][6] == pytest.approx(controller.calibration.gripper_open)
    loaded = LeRobotDataset("local/example6_recording", root=tmp_path / "dataset", video_backend="pyav")
    assert loaded.num_episodes == 2
    sample = loaded[0]
    assert sample["observation.images.front"].shape == (3, 64, 64)
    assert any(np.allclose(sample["action"].numpy(), np.rad2deg(q), atol=1e-5) for q in commands)


def test_record_loop_cancellation_is_not_consumed_between_phases():
    from lerobot.scripts.lerobot_record import record_loop

    robot, processor = MagicMock(), MagicMock()
    events = {"stop_recording": True, "exit_early": True}
    for _ in range(2):  # An episode followed by a reset must both stop immediately.
        record_loop(robot, events, 30, processor, processor, processor, control_time_s=10)
    robot.get_observation.assert_not_called()
    robot.send_action.assert_not_called()
    assert events["stop_recording"]


@pytest.mark.parametrize("background", [False, True])
def test_default_teleop_attaches_same_example_and_stops(example, tmp_path, background):
    module, controller, _ = example
    teleop = JoyconTeleop(JoyconTeleopConfig(software_calibration=CALIBRATION, calibration_dir=tmp_path))
    robot = RebotB601Follower(
        RebotB601FollowerRobotConfig(
            port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path
        )
    )
    validate_joycon_robot(teleop, robot)
    assert robot._defer_example_home
    robot._example_sdk = FakeArm(controller.init_qpos)
    with patch.object(module, "Example6Controller", return_value=controller):
        teleop.connect()
    with patch.object(controller, "step", return_value=(controller.init_qpos, 0)):
        teleop.start_control(robot, background=background)
        assert (teleop._runtime.thread.ident is not None) is background
        assert len(teleop.get_action()) == 7
        teleop.stop_control()
    assert robot._joycon_runtime is None
    teleop.disconnect()


def test_refuses_old_extra_limits(tmp_path):
    with pytest.raises(ValueError, match="remove max_relative_target"):
        RebotB601FollowerRobotConfig(
            port="/dev/null", software_calibration=CALIBRATION, max_relative_target=5
        )
    teleop = JoyconTeleop(
        JoyconTeleopConfig(software_calibration=CALIBRATION, gripper_speed_rad_s=1, calibration_dir=tmp_path)
    )
    with pytest.raises(ValueError, match="remove legacy"):
        teleop.connect()


def test_partial_camera_connect_failure_closes_sdk(example, tmp_path):
    module, controller, _ = example
    sdk = FakeArm(controller.init_qpos)
    sdk.MotorController = SimpleNamespace(serial_=MagicMock())
    camera = MagicMock()
    camera.is_connected = False
    camera.connect.side_effect = RuntimeError("camera unavailable")
    config = RebotB601FollowerRobotConfig(
        port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path
    )
    with patch(
        "lerobot.robots.rebot_b601_follower.rebot_b601_follower.make_cameras_from_configs",
        return_value={"front": camera},
    ):
        robot = RebotB601Follower(config)
    with (
        patch.object(module, "ReBot", return_value=sdk),
        pytest.raises(RuntimeError, match="camera unavailable"),
    ):
        robot.connect()
    assert sdk.disabled
    sdk.MotorController.serial_.close.assert_called_once()
    assert not robot.is_connected


@pytest.mark.parametrize("rollout_wrapper", [False, True])
def test_actual_sdk_arguments_are_used_without_gain_reimplementation(tmp_path, rollout_wrapper):
    module = load_example6()
    sdk = module.ReBot(if_connect_directly=False, calibration_path=CALIBRATION)
    sdk.Motors = list(range(7))
    sdk.MotorController = MagicMock()
    robot = RebotB601Follower(
        RebotB601FollowerRobotConfig(
            port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path
        )
    )
    robot._example_sdk = sdk
    target = sdk.home_pose.copy()
    target[0] = 0.3
    target[6] = sdk.control_qlimit[0][6]
    assert sdk.kp[6] == 2.0
    assert sdk.kd[6] == 1.0
    sender = robot
    if rollout_wrapper:
        from lerobot.rollout.robot_wrapper import ThreadSafeRobot
        sender = ThreadSafeRobot(robot)
    standalone = module.ReBot(if_connect_directly=False, calibration_path=CALIBRATION)
    standalone.Motors = list(range(7))
    standalone.MotorController = MagicMock()
    with (
        patch.object(sdk, "update_motors_status"),
        patch.object(standalone, "update_motors_status"),
        patch("ReBot_SDK.time.monotonic") as clock,
    ):
        # Compare independent histories at equal elapsed times, including arm smoothing.
        for tick in range(400):
            clock.return_value = 100.0 + tick * .01
            standalone.MotorController.reset_mock()
            sdk.MotorController.reset_mock()
            standalone.ReBot_Control(target, smooth=True)
            sender.send_action(
                {
                    f"{name}.pos": float(value)
                    for name, value in zip(robot.motor_names, np.rad2deg(target), strict=True)
                }
            )
            for actual, reference in zip(
                sdk.MotorController.controlMIT.call_args_list,
                standalone.MotorController.controlMIT.call_args_list,
                strict=True,
            ):
                assert actual.kwargs == pytest.approx(reference.kwargs)
    gripper_command = sdk.MotorController.controlMIT.call_args.kwargs
    assert gripper_command["tau"] == 0.0  # Feedforward only, not total motor torque.
    assert gripper_command["kp"] == 2.0
    assert gripper_command["kd"] == 1.0
    assert gripper_command["q"] == pytest.approx(
        target[6] * sdk.calibration.signs[6] + sdk.calibration.offsets[6]
    )
    assert sdk.motor_type == [0, 0, 0, 1, 1, 1, 1]


def test_examples_robot_returns_protected_action(tmp_path):
    module = load_example6()
    sdk = module.ReBot(if_connect_directly=False, calibration_path=CALIBRATION)
    sdk.Motors = list(range(7))
    sdk.MotorController = MagicMock()
    robot = RebotB601Follower(RebotB601FollowerRobotConfig(
        port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path))
    robot._example_sdk = sdk
    sdk.motors_pose[6] = -.5
    action = dict(zip(
        (f"{name}.pos" for name in robot.motor_names), np.rad2deg(sdk.home_pose), strict=True))
    with patch.object(sdk, "update_motors_status"):
        for torque in (-1.2, .2, .2):
            sdk.motors_torque[6] = torque
            sent = robot.send_action(action)
            assert sent["gripper.pos"] == pytest.approx(np.rad2deg(-.5))
        action["gripper.pos"] = np.rad2deg(-.6)
        assert robot.send_action(action)["gripper.pos"] == pytest.approx(np.rad2deg(-.6))


def test_joycon_uses_protected_target_as_next_ramp_origin(example):
    module, controller, _ = example
    sdk = module.ReBot(if_connect_directly=False, calibration_path=CALIBRATION)
    sdk.Motors = list(range(7))
    sdk.MotorController = MagicMock()
    controller.arm = sdk
    controller.t = module.STARTUP_HOME_CYCLES + 1
    controller.last_sent[6] = -.5
    sdk.motors_pose[6] = -.5
    with (
        patch.object(sdk, "update_motors_status"),
        patch.object(module, "rebot_IK", return_value=(controller.init_qpos[:6], False)),
        patch.object(controller, "_simulate"),
        patch.object(module.time, "monotonic", return_value=10.),
    ):
        controller.joycon.get_control.return_value = (np.zeros(6), sdk.calibration.gripper_close, 0)
        for torque in (1.2, .2, .2):
            controller._gripper_tick = 9.99
            sdk.motors_torque[6] = torque
            sent, _ = controller.step()
            assert sent[6] == pytest.approx(-.5)
            assert controller.last_sent[6] == pytest.approx(-.5)
        controller.joycon.get_control.return_value = (np.zeros(6), sdk.calibration.gripper_open, 0)
        controller._gripper_tick = 9.99
        sent, _ = controller.step()
        assert sent[6] < -.5


def test_examples_config_roundtrip_does_not_materialize_legacy_motor_tuning():
    import draccus

    config = RebotB601FollowerRobotConfig(port="/dev/null", software_calibration=CALIBRATION)
    restored = draccus.decode(RebotB601FollowerRobotConfig, draccus.encode(config))
    assert restored.control_backend == "examples"
    assert restored.mit_kp is None and restored.mit_kd is None


def test_late_worker_error_prevents_normal_home(tmp_path):
    teleop = JoyconTeleop(JoyconTeleopConfig(calibration_dir=tmp_path))
    runtime = MagicMock()
    runtime.snapshot.side_effect = RuntimeError("worker failed")
    teleop._runtime = runtime
    with pytest.raises(RuntimeError, match="worker failed"):
        teleop.stop_control(check_error=True)
    runtime.stop.assert_called_once()
    assert runtime.robot._joycon_runtime is None
    assert teleop._runtime is None
    teleop.stop_control()  # Finally cleanup remains safe after propagation.


def test_real_sdk_home_after_slow_camera_initialization(tmp_path):
    module = load_example6()
    sdk = module.ReBot(if_connect_directly=False, calibration_path=CALIBRATION)
    sdk.is_connected_flag = True
    sdk.MotorController = MagicMock()
    sdk.Motors = [MagicMock() for _ in range(7)]
    for motor in sdk.Motors:
        motor.feedback_time = 100.0
        motor.getPosition.return_value = 0.0
        motor.getVelocity.return_value = 0.0
        motor.getTorque.return_value = 0.0
    clock = [100.0]
    camera = MagicMock()
    camera.is_connected = False
    def connect_camera():
        clock[0] += 3.0
        camera.is_connected = True
    camera.connect.side_effect = connect_camera
    def receive():
        if clock[0] >= 103.015:
            sdk.Motors[6].feedback_time = clock[0]
    sdk.MotorController.recv.side_effect = receive
    def sleep(seconds):
        clock[0] += seconds
    def send(*args, **kwargs):
        assert sdk.Motors[6].feedback_time >= 103.015
    sdk.MotorController.controlMIT.side_effect = send
    config = RebotB601FollowerRobotConfig(
        port="/dev/null", software_calibration=CALIBRATION, calibration_dir=tmp_path
    )
    with patch(
        "lerobot.robots.rebot_b601_follower.rebot_b601_follower.make_cameras_from_configs",
        return_value={"front": camera},
    ):
        robot = RebotB601Follower(config)
    with (
        patch.object(module, "ReBot", return_value=sdk),
        patch("ReBot_SDK.time.monotonic", side_effect=lambda: clock[0]),
        patch("ReBot_SDK.time.sleep", side_effect=sleep),
    ):
        robot.connect()
    assert robot.is_connected
    assert sdk.MotorController.controlMIT.call_count == 7
    sdk.MotorController.disable.assert_not_called()
    camera.disconnect.assert_not_called()
    assert 103.015 <= clock[0] <= 103.021
