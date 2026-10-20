"""No serial/HID/window access: verify actual subprocess logging and SDK events."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import test_rebot_keyboard_real as fixtures
from runtime_log import RuntimeLog


class RuntimeLogTests(unittest.TestCase):
    def test_sdk_logs_real_mocked_can_cycle_and_snapshots(self):
        arm, serial = fixtures.SDKCommandTests().make_arm()
        with tempfile.TemporaryDirectory() as directory:
            session = RuntimeLog(directory)
            try:
                with patch("ReBot_SDK.record_event", side_effect=session.emit):
                    with patch("time.monotonic", return_value=10.):
                        for i, motor in enumerate(arm.Motors):
                            motor.recv_data(
                                arm.home_pose[i] * arm.calibration.signs[i] + arm.calibration.offsets[i],
                                0., 0.)
                        arm.Go_Home_Pose()
                    target = arm.home_pose.copy()
                    target[6] = arm.calibration.gripper_open
                    with patch("time.monotonic", return_value=10.01):
                        arm.ReBot_Control(target)
                    target[:] = 123  # Buffered events must retain the original request.
            finally:
                session.close()
            self.assertIsNone(session.error)
            rows = [json.loads(line) for line in (session.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual(len(serial.frames), 28)
            self.assertEqual(sum(r["event"] == "joint_command_returned" for r in rows), 14)
            commands = [r for r in rows if r["event"] == "gripper_command_attempt"]
            self.assertEqual(len(commands), 2)
            self.assertAlmostEqual(commands[1]["elapsed_s"], .01)
            self.assertEqual(commands[1]["feedforward_torque"], 0)
            self.assertEqual(commands[1]["control"], "motor_position")
            self.assertEqual(commands[1]["kd"], 1.0)
            self.assertEqual(commands[1]["target"], arm.calibration.gripper_open)
            requests = [r for r in rows if r["event"] == "control_request"]
            self.assertEqual(requests[1]["target"][6], arm.calibration.gripper_open)
            self.assertEqual(rows[-1]["event"], "session_end")

    def test_console_exception_and_shutdown_are_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-c",
                 "from runtime_log import start_runtime_log, record_event\n"
                 "start_runtime_log()\nstart_runtime_log()\n"
                 "print('中文输出')\nrecord_event('sample', target=[1, 2, 3])\n"
                 "raise RuntimeError('diagnostic failure')"],
                cwd=Path(__file__).parent,
                env={**os.environ, "REBOT_LOG_DIR": directory},
                text=True, capture_output=True,
            )
            self.assertNotEqual(result.returncode, 0)
            folders = list(Path(directory).iterdir())
            self.assertEqual(len(folders), 1)
            console = (folders[0] / "console.log").read_text()
            self.assertIn("中文输出", console)
            self.assertIn("RuntimeError: diagnostic failure", console)
            rows = [json.loads(line) for line in (folders[0] / "events.jsonl").read_text().splitlines()]
            self.assertEqual(rows[0]["event"], "session_start")
            self.assertEqual(rows[-1]["event"], "session_end")
            self.assertEqual(next(row for row in rows if row["event"] == "sample")["target"], [1, 2, 3])


if __name__ == "__main__":
    unittest.main()
