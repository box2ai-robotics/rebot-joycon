"""Offline regression: real script loops/model, fake keyboard, window and CAN."""
import contextlib
import io
import json
from pathlib import Path
import runpy
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import mujoco.viewer
import numpy as np

import ReBot_Kinematics as kinematics
import ReBot_SDK as sdk
from Motor_CAN import DM_Motor_Type, Motor, MotorControl


EXAMPLES = Path(__file__).resolve().parent


class GripperCalibrationTests(unittest.TestCase):
    def calibration_at(self, directory, closed, drive_mode=0):
        config = json.loads((EXAMPLES / "calibration/rebot.json").read_text())
        snapshot = json.loads((EXAMPLES / "calibration/rebot.encoders.json").read_text())
        config["gripper"]["drive_mode"] = drive_mode
        config["gripper"]["range_max"] = 0
        snapshot["gripper"]["position_rad"] = closed
        path = Path(directory) / "rebot.json"
        path.write_text(json.dumps(config))
        path.with_suffix(".encoders.json").write_text(json.dumps(snapshot))
        return sdk.ReBotCalibration(path)

    def test_measured_close_is_not_clipped_to_nominal_zero_in_either_direction(self):
        with tempfile.TemporaryDirectory() as directory:
            for closed in (.7970931529998779, -.085641):
                for mode, sign in ((0, 1), (1, -1)):
                    with self.subTest(closed=closed, mode=mode):
                        calibration = self.calibration_at(directory, closed, mode)
                        self.assertAlmostEqual(calibration.gripper_close, sign * closed)
                        self.assertAlmostEqual(calibration.gripper_open, sign * np.deg2rad(-270))
                        self.assertEqual(calibration.offsets[6], 0.)
                        np.testing.assert_allclose(
                            calibration.limits[:, 6],
                            sorted([calibration.gripper_open, calibration.gripper_close]))
                        q = np.zeros(7)
                        q[6] = calibration.gripper_close
                        self.assertAlmostEqual(calibration.to_motor(q)[6], closed)
                        np.testing.assert_allclose(calibration.from_motor(calibration.to_motor(q)), q)

    def test_invalid_closed_endpoint_is_rejected_instead_of_silently_clipped(self):
        with tempfile.TemporaryDirectory() as directory:
            for closed in (float("nan"), float("inf"), -4.8, np.deg2rad(-270), 12.6):
                with self.subTest(closed=closed), self.assertRaises(ValueError):
                    self.calibration_at(directory, closed)


class KeyboardRealTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = kinematics.get_robot("rebot_b601_dm")
        cls.calibration = sdk.ReBotCalibration()
        cls.home = np.r_[kinematics.get_home_q(cls.robot), cls.calibration.gripper_close]

    def run_example(self, number, *, frames=60, period=.01, events=None, errors=None, ik=None, overload=False):
        state = SimpleNamespace(frame=-1, now=0., trace=[], commands=[], homes=[])
        listener = MagicMock()
        arm = MagicMock()
        arm.motors_pose = self.home.copy() - (np.zeros(7) if errors is None else errors)
        arm.motors_torque = np.zeros(7)
        arm.target_pose = self.home.copy()
        protection = sdk.GripperTorqueProtection(lambda *args, **kwargs: None, "/dev/null")
        state.applied = []
        arm.kp_control = [80., 80., 80., 19.2, 19.2, 19.2, 1.6]

        def send(q):
            state.commands.append((state.frame, np.asarray(q).copy()))
            arm.target_pose = np.asarray(q).copy()
            if overload:
                arm.target_pose[6] = protection.apply(
                    float(q[6]), float(self.home[6] - .01), 1.2 if state.frame == 40 else .1)
            state.applied.append((state.frame, arm.target_pose.copy()))
            state.trace.append((state.frame, "send"))

        def home():
            arm.target_pose = self.home.copy()
            state.homes.append(state.frame)
            state.trace.append((state.frame, "send"))

        arm.ReBot_Control.side_effect = send
        arm.Go_Home_Pose.side_effect = home

        def make_listener(**callbacks):
            state.callbacks = callbacks
            return listener

        viewer = MagicMock()
        viewer.__enter__.return_value = viewer
        viewer.sync.side_effect = lambda: state.trace.append((state.frame, "sync"))

        def running():
            state.frame += 1
            state.now = state.frame * period
            if state.frame >= frames:
                return False
            for kind, key in (events or {}).get(state.frame, []):
                state.callbacks[kind](SimpleNamespace(char=key))
            return True

        viewer.is_running.side_effect = running
        original_exists = Path.exists

        def exists(path):
            return True if str(path) in ("/dev/ttyACM0", "/dev/ttyACM1") else original_exists(path)

        filename = next(EXAMPLES.glob(f"{number}_ReBot_B601_DM_Key_*_REAL.py"))
        state.exception = None
        output = io.StringIO()
        with (
            patch.dict(sys.modules, pynput=SimpleNamespace(
                keyboard=SimpleNamespace(Listener=make_listener))),
            patch.object(sdk, "ReBot", return_value=arm),
            patch.object(mujoco.viewer, "launch_passive", return_value=viewer),
            patch("serial.Serial", side_effect=AssertionError("No hardware in offline tests")),
            patch("os.popen", return_value=SimpleNamespace(read=lambda: "serial 0000000")),
            patch.object(Path, "exists", exists),
            patch("time.monotonic", side_effect=lambda: state.now),
            patch("time.sleep"),
            patch.object(kinematics, "rebot_IK", wraps=kinematics.rebot_IK) if ik is None
            else patch.object(kinematics, "rebot_IK", side_effect=ik),
            contextlib.redirect_stdout(output),
        ):
            try:
                state.scope = runpy.run_path(str(filename))
            except Exception as exc:
                state.exception = exc
        state.arm, state.viewer, state.output = arm, viewer, output.getvalue()
        listener.stop.assert_called_once()
        arm.ReBot_Disable.assert_called_once()
        arm.MotorController.serial_.close.assert_called_once()
        return state

    def assert_ok(self, result):
        if result.exception is not None:
            raise result.exception

    def test_keyboard_targets_follow_protection_and_reverse_without_backlog(self):
        for number, forward, reverse in ((4, "u", "7"), (5, "3", "1")):
            result = self.run_example(number, overload=True, frames=55, events={
                40: [("on_press", forward)],
                50: [("on_release", forward), ("on_press", reverse)],
            })
            self.assert_ok(result)
            applied = dict(result.applied)
            boundary = self.home[6] - .01
            for frame in range(40, 50):
                self.assertAlmostEqual(applied[frame][6], boundary)
            self.assertGreater(applied[50][6], boundary)
            requests = dict(result.commands)
            self.assertAlmostEqual(requests[49][6], boundary - .02)

    def test_4_startup_matches_learning_model_without_error_gate(self):
        for errors in (np.zeros(7), np.full(7, .3)):
            result = self.run_example(
                4, frames=50, errors=errors,
                events={40: [("on_press", "u")], 44: [("on_release", "u")]})
            self.assert_ok(result)
            self.assertEqual(result.homes[:40], [-1] + list(range(39)))
            self.assertEqual([frame for frame, _ in result.commands], list(range(40, 50)))
            q = np.array([q for _, q in result.commands])
            np.testing.assert_allclose(q[:4, 6], self.home[6] - .02 * np.arange(1, 5))
            np.testing.assert_allclose(q[4:, 6], q[3, 6])
            self.assertEqual(result.homes[40:], [50] * 100)

    def test_4_time_based_motion_pause_clamp_limits_and_reset(self):
        event = {40: [("on_press", "u")]}
        fast = self.run_example(4, frames=60, period=.005, events=event)
        slow = self.run_example(4, frames=50, period=.01, events=event)
        paused = self.run_example(4, frames=41, period=.5, events=event)
        for result in (fast, slow, paused):
            self.assert_ok(result)
        np.testing.assert_allclose(fast.commands[-1][1], slow.commands[-1][1])
        self.assertAlmostEqual(paused.commands[-1][1][6], self.home[6] - .04)
        reset_frame = 40 + int(np.ceil(abs(self.calibration.gripper_open - self.home[6]) / .02)) + 10
        result = self.run_example(
            4, frames=reset_frame + 70, events={40: [("on_press", "u")], reset_frame: [
                ("on_release", "u"), ("on_press", "0")]})
        self.assert_ok(result)
        self.assertAlmostEqual(dict(result.commands)[reset_frame - 1][6], self.calibration.gripper_open)
        self.assertIn(reset_frame, result.homes)
        np.testing.assert_allclose(result.commands[-1][1], self.home)

    def test_5_failed_ik_keeps_sending_and_gripper_keeps_moving(self):
        failed_ik = MagicMock(return_value=(np.zeros(6), False))
        result = self.run_example(
            5, frames=70, events={40: [("on_press", "r"), ("on_press", "3")]}, ik=failed_ik)
        self.assert_ok(result)
        self.assertEqual([frame for frame, _ in result.commands], list(range(40, 70)))
        self.assertEqual(failed_ik.call_count, 30)
        q = np.array([q for _, q in result.commands])
        np.testing.assert_allclose(q[:, :6], np.tile(self.home[:6], (30, 1)))
        np.testing.assert_allclose(q[:, 6], self.home[6] - .02 * np.arange(1, 31))
        for call in failed_ik.call_args_list:
            np.testing.assert_allclose(call.args[0], self.home[:6])
        for frame, kind in result.trace:
            if kind == "sync" and frame != 39:
                self.assertLess(result.trace.index((frame, "send")), result.trace.index((frame, "sync")))
        self.assertLess(result.viewer.sync.call_count, 25)

    def test_5_gripper_only_skips_ik_and_zero_resets_both_targets(self):
        ik = MagicMock(side_effect=AssertionError("Gripper-only input must skip IK"))
        result = self.run_example(
            5, frames=100, events={40: [("on_press", "3")], 45: [
                ("on_release", "3"), ("on_press", "0")]}, ik=ik)
        self.assert_ok(result)
        ik.assert_not_called()
        self.assertTrue(any(frame >= 85 for frame, _ in result.commands))
        np.testing.assert_allclose(result.commands[-1][1], self.home)
        np.testing.assert_allclose(result.scope["target_gpos"], result.scope["init_gpos"])

    def test_5_real_ik_small_steps_are_continuous(self):
        result = self.run_example(5, frames=65, events={40: [("on_press", "r")]})
        self.assert_ok(result)
        q = np.array([q for _, q in result.commands])
        positions = np.array([kinematics.rebot_FK(row[:6], self.robot)[:3] for row in q])
        self.assertTrue(np.all(np.diff(positions[:, 2]) > .001))
        self.assertLess(np.max(np.abs(np.diff(q[:, :6], axis=0))), .1)

    def test_5_fault_does_not_send_exit_home(self):
        result = self.run_example(
            5, frames=60, events={40: [("on_press", "r")]},
            ik=MagicMock(side_effect=RuntimeError("test IK failure")))
        self.assertIn("test IK failure", str(result.exception))
        self.assertEqual(result.homes, [-1] + list(range(39)))

    def test_both_keyboards_open_then_return_to_measured_closed_endpoint(self):
        measured_close = json.loads(
            (EXAMPLES / "calibration/rebot.encoders.json").read_text())["gripper"]["position_rad"]
        travel_frames = int(np.ceil(abs(self.calibration.gripper_open - self.home[6]) / .02)) + 10
        reverse_frame = 40 + travel_frames
        release_frame = reverse_frame + travel_frames
        for number, opening, closing in ((4, "u", "7"), (5, "3", "1")):
            with self.subTest(number=number):
                result = self.run_example(number, frames=release_frame + 10, events={
                    40: [("on_press", opening)],
                    reverse_frame: [("on_release", opening), ("on_press", closing)],
                    release_frame: [("on_release", closing)],
                })
                self.assert_ok(result)
                commands = dict(result.commands)
                self.assertAlmostEqual(commands[reverse_frame - 1][6], self.calibration.gripper_open)
                self.assertAlmostEqual(commands[release_frame + 9][6], measured_close * self.calibration.signs[6])


class StandaloneLifecycleTests(unittest.TestCase):
    def test_6_matches_learning_model_normal_and_interrupted_cleanup(self):
        from importlib import import_module
        module = import_module("6_ReBot_B601_DM_Joycon_Gpos_REAL")
        for failure in (None, KeyboardInterrupt(), RuntimeError("control failed")):
            with self.subTest(failure=type(failure).__name__):
                arm, controller = MagicMock(), MagicMock()
                with (
                    patch.object(module, "Example6Controller", return_value=controller),
                    patch.object(module, "ReBot", return_value=arm),
                    patch.object(module, "run_control_loop", side_effect=failure),
                    patch("subprocess.run", return_value=SimpleNamespace(stdout="serial 0000000")),
                    patch("serial.Serial", side_effect=AssertionError("No hardware")),
                    contextlib.redirect_stdout(io.StringIO()),
                ):
                    if isinstance(failure, RuntimeError):
                        with self.assertRaisesRegex(RuntimeError, "control failed"):
                            module.main()
                    else:
                        module.main()
                controller.attach.assert_called_once_with(arm)
                self.assertEqual(arm.Go_Home_Pose.call_count, 100 if failure is None else 0)
                arm.ReBot_Disable.assert_called_once()
                arm.MotorController.serial_.close.assert_called_once()
                controller.close_joycon.assert_called_once()


class FakeSerial:
    is_open = False

    def __init__(self):
        self.frames = []

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def read_all(self):
        return b""

    def write(self, data):
        self.frames.append(data)
        return len(data)


class SDKCommandTests(unittest.TestCase):



    @staticmethod
    def decode_mit(frame):
        data = frame[21:29]
        return {
            "q": int.from_bytes(data[:2], "big") * 25. / 65535 - 12.5,
            "dq": ((data[2] << 4) | (data[3] >> 4)) * 60. / 4095 - 30.,
            "kp": (((data[3] & 15) << 8) | data[4]) * 500. / 4095,
            "kd": ((data[5] << 4) | (data[6] >> 4)) * 5. / 4095,
            "tau": (((data[6] & 15) << 8) | data[7]) * 20. / 4095 - 10.,
        }

    def test_gripper_uses_motor_position_pd_for_both_endpoints_and_directions(self):
        arm, serial = self.make_arm()
        for sign in (-1, 1):
            arm.calibration.signs[6] = sign
            for target in (arm.calibration.gripper_open, arm.calibration.gripper_close):
                with self.subTest(sign=sign, target=target):
                    arm.target_pose[6] = target
                    arm._send_joint_command(6)
                    command = self.decode_mit(serial.frames[-1])
                    self.assertAlmostEqual(command["q"], target * sign, delta=25. / 65535)
                    self.assertAlmostEqual(command["kp"], 2., delta=500. / 4095)
                    self.assertAlmostEqual(command["kd"], 1., delta=5. / 4095)
                    self.assertAlmostEqual(command["dq"], 0., delta=60. / 4095)
                    self.assertAlmostEqual(command["tau"], 0., delta=20. / 4095)
                    self.assertGreater(command["kp"], 0)
                    self.assertGreater(command["kd"], 0)

    def test_gripper_command_does_not_depend_on_host_feedback_or_loop_period(self):
        arm, serial = self.make_arm()
        arm.target_pose[6] = arm.calibration.gripper_close
        for stamp, position, velocity in ((0., -4., 0.), (.01, .5, 5.), (.5, 1.2, -5.)):
            arm.motors_pose[6], arm.motors_speed[6] = position, velocity
            with patch("time.monotonic", return_value=stamp):
                arm._send_joint_command(6)
        self.assertEqual(serial.frames[0], serial.frames[1])
        self.assertEqual(serial.frames[1], serial.frames[2])

    def test_home_uses_motor_pd_and_preserves_six_axis_gains(self):
        arm, serial = self.make_arm()
        with patch.object(arm, "update_motors_status"):
            arm.Go_Home_Pose()
        for idx, frame in enumerate(serial.frames):
            command = self.decode_mit(frame)
            self.assertAlmostEqual(command["kp"], arm.kp[idx] * .8, delta=500. / 4095)
            self.assertAlmostEqual(command["kd"], arm.kd[idx], delta=5. / 4095)
            self.assertAlmostEqual(command["q"],
                                   arm.home_pose[idx] * arm.calibration.signs[idx] + arm.calibration.offsets[idx],
                                   delta=25. / 65535)

    def test_overload_retracts_target_and_latches_both_directions(self):
        for direction in (-1, 1):
            for feedback_torque in (-1.01, 1.01):
                arm, serial = self.make_arm()
                arm.calibration.signs[6] = direction
                arm.calibration.offsets[6] = .3
                arm.motors_pose[6] = -.5
                arm.motors_torque[6] = feedback_torque
                arm.target_pose[6] = -.5 + direction
                arm._send_joint_command(6)
                self.assertEqual(arm.target_pose[6], -.5)
                command = self.decode_mit(serial.frames[-1])
                self.assertAlmostEqual(command["q"], -.5 * direction + .3, delta=25. / 65535)
                # Returning below threshold must not resume the old goal.
                arm.motors_torque[6] = .2
                for _ in range(3):
                    arm.target_pose[6] = -.5 + direction * .1
                    arm._send_joint_command(6)
                    self.assertEqual(arm.target_pose[6], -.5)
                # Escape is allowed immediately, even before torque falls.
                arm.motors_torque[6] = 1.2
                arm.target_pose[6] = -.5 - direction * .1
                arm._send_joint_command(6)
                self.assertAlmostEqual(arm.target_pose[6], -.5 - direction * .1)
                self.assertIsNotNone(arm._gripper_protection.hold)
                arm.motors_torque[6] = .2
                arm._send_joint_command(6)
                self.assertIsNone(arm._gripper_protection.hold)

    def test_overload_boundary_does_not_creep_forward(self):
        arm, _ = self.make_arm()
        arm.motors_torque[6] = 1.1
        for position, expected in ((0., 0.), (.05, 0.), (-.05, -.05)):
            arm.target_pose[6] = 1.
            arm.motors_pose[6] = position
            arm._send_joint_command(6)
            self.assertEqual(arm.target_pose[6], expected)

    def test_torque_threshold_and_invalid_feedback(self):
        arm, _ = self.make_arm()
        for measured in (-1., 1.):
            arm.target_pose[6] = 1.
            arm.motors_torque[6] = measured
            arm._send_joint_command(6)
            self.assertEqual(arm.target_pose[6], 1.)
        for measured in (np.nan, np.inf, -np.inf):
            arm.motors_torque[6] = measured
            with patch.object(arm, "ReBot_Disable") as disable:
                with self.assertRaises(ValueError):
                    arm._send_joint_command(6)
                disable.assert_called_once()

    def test_control_and_home_share_protection_without_changing_six_axes(self):
        arm, serial = self.make_arm()
        arm.motors_pose[6] = -.5
        arm.motors_torque[6] = -1.2
        with patch.object(arm, "update_motors_status"):
            arm.ReBot_Control(arm.home_pose)
            arm.motors_torque[6] = .1
            arm.Go_Home_Pose()
        self.assertEqual(arm.target_pose[6], -.5)
        for idx in range(6):
            command = self.decode_mit(serial.frames[idx])
            self.assertAlmostEqual(command["q"],
                                   arm.home_pose[idx] * arm.calibration.signs[idx] + arm.calibration.offsets[idx],
                                   delta=25. / 65535)

    def make_arm(self):
        arm = sdk.ReBot(if_connect_directly=False)
        serial = FakeSerial()
        arm.MotorController = MotorControl(serial)
        arm.Motors = [
            Motor(DM_Motor_Type.DM4340 if i < 3 else DM_Motor_Type.DM4310, i + 1, i + 17)
            for i in range(7)]
        for motor in arm.Motors:
            arm.MotorController.addMotor(motor)
        return arm, serial

    def test_one_control_cycle_queries_seven_then_sends_seven_mit_frames(self):
        arm, serial = self.make_arm()
        with patch("time.monotonic", return_value=10.), patch("Motor_CAN.monotonic", return_value=10.):
            for i, motor in enumerate(arm.Motors):
                motor.recv_data(
                    arm.home_pose[i] * arm.calibration.signs[i] + arm.calibration.offsets[i], 0., 0.)
            arm.ReBot_Control(arm.home_pose)
        self.assertEqual(len(serial.frames), 14)
        self.assertTrue(all(len(frame) == 30 for frame in serial.frames))
        ids = [int.from_bytes(frame[13:15], "little") for frame in serial.frames]
        self.assertEqual(ids, [0x7FF] * 7 + list(range(1, 8)))
        self.assertEqual([frame[21] for frame in serial.frames[:7]], list(range(1, 8)))
        self.assertTrue(all(frame[23] == 0xCC for frame in serial.frames[:7]))


    def test_home_and_control_preserve_positive_measured_close(self):
        arm, _ = self.make_arm()
        measured_close = json.loads(
            (EXAMPLES / "calibration/rebot.encoders.json").read_text())["gripper"]["position_rad"]
        with patch.object(arm, "update_motors_status"), patch("time.monotonic", return_value=1.):
            arm.Go_Home_Pose()
            self.assertAlmostEqual(arm.target_pose[6], measured_close * arm.calibration.signs[6])
            target = arm.home_pose.copy()
            target[6] = 100.
            arm.ReBot_Control(target)
            self.assertAlmostEqual(arm.target_pose[6], arm.control_qlimit[1][6])
            target[6] = -100.
            arm.ReBot_Control(target)
            self.assertAlmostEqual(arm.target_pose[6], arm.control_qlimit[0][6])


    def test_stale_feedback_still_disables_without_sending_movement(self):
        arm, _ = self.make_arm()
        clock = SimpleNamespace(now=10.)
        for motor in arm.Motors:
            motor.feedback_time = 0.
        with (
            patch("time.monotonic", side_effect=lambda: clock.now),
            patch("time.sleep", side_effect=lambda dt: setattr(clock, "now", clock.now + dt)),
            patch.object(arm.MotorController, "controlMIT") as send,
            patch.object(arm, "ReBot_Disable", wraps=arm.ReBot_Disable) as disable,
        ):
            with self.assertRaisesRegex(RuntimeError, "反馈缺失或超时"):
                arm.ReBot_Control(arm.home_pose)
        send.assert_not_called()
        disable.assert_called_once()
        self.assertIsNone(arm._last_gripper_command_time)


    def test_invalid_gripper_state_still_disables(self):
        arm, _ = self.make_arm()
        arm.motors_speed[6] = np.nan
        with patch.object(arm, "ReBot_Disable") as disable:
            with self.assertRaises(ValueError):
                arm._send_joint_command(6)
        disable.assert_called_once()


if __name__ == "__main__":
    unittest.main()
