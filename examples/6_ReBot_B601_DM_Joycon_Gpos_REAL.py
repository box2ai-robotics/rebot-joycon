# code by Boxjod 2025.12.01 Box2AI-Robotics copyright 盒桥智能 版权所有
"""Example 6 and LeRobot share this exact controller, simulation and SDK path.

Importing the module does not open HID/serial devices. main() runs the original viewer.
"""

if __name__ == "__main__":
    from runtime_log import start_runtime_log
    start_runtime_log()

import os
import time
from pathlib import Path
from importlib import import_module

import mujoco
import numpy as np

from ReBot_Kinematics import create_rebot_b601_dm, rebot_IK, rebot_FK, get_home_q
from ReBot_SDK import ReBot
from rebot_port import select_port

ReBotCalibration = import_module("0_calibrate_rebot").ReBotCalibration
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_INCREMENT = 0.01  # Retained from example 6; not used by its Cartesian loop.
POSITION_INSERMENT = 0.0008  # Likewise not a Joy-Con speed setting.
CONTROL_PERIOD_S = 0.001
STARTUP_HOME_CYCLES = 40
EXIT_HOME_CYCLES = 100
SIMULATION_STEPS = 2
GRIPPER_TRAVEL_TIME_S = 1.0  # Full calibrated stroke; shared by standalone and LeRobot.
CONTROL_GLIMIT = [[0.000, -0.450, 0.00, -3.1, -3.1, -3.1],
                  [0.450, 0.450, 0.680, 3.1, 3.1, 3.1]]
DOF_SPEED = [2 / 3, 2 / 3, 2 / 3, 1, 1, 1]
XML_PATH = Path(__file__).resolve().parents[1] / "URDF/reBot_B601_DM/scene.xml"


class Example6Controller:
    """One step is the original example's loop body, with optional viewer rendering."""

    def __init__(self, calibration_path=None):
        from joyconrobotics import JoyconRobotics

        self.calibration = ReBotCalibration() if calibration_path is None else ReBotCalibration(calibration_path)
        self.robot = create_rebot_b601_dm(calibration=self.calibration)
        self.control_qlimit = self.calibration.control_limits(self.robot.qlim)
        self.mjmodel = mujoco.MjModel.from_xml_path(str(XML_PATH))
        self.mjdata = mujoco.MjData(self.mjmodel)
        self.qpos_indices = np.array([self.mjmodel.jnt_qposadr[self.mjmodel.joint(n).id] for n in JOINT_NAMES])
        self.finger_indices = [self.mjmodel.joint(n).qposadr[0] for n in ("finger_left", "finger_right")]
        self.init_qpos = np.r_[get_home_q(self.robot), self.calibration.gripper_close]
        self.target_qpos = self.init_qpos.copy()
        self.target_qpos_last = self.init_qpos.copy()
        self.last_sent = self.init_qpos.copy()
        self.init_gpos = rebot_FK(self.init_qpos[:6], robot=self.robot)
        self.mjdata.qpos[self.qpos_indices] = self.init_qpos[:6]
        self.mjdata.qpos[self.finger_indices] = 0.0
        self.mjdata.ctrl[:] = np.r_[self.init_qpos[:6], 0.0]
        mujoco.mj_forward(self.mjmodel, self.mjdata)
        # Joy-Con reports calibrated open/close angles; the controller interpolates them.
        self.joycon = JoyconRobotics(
            device="right", glimit=[row.copy() for row in CONTROL_GLIMIT],
            dof_speed=DOF_SPEED.copy(), offset_position_m=self.init_gpos[:3],
            gripper_close=self.calibration.gripper_close,
            gripper_open=self.calibration.gripper_open,
            gripper_state=self.calibration.gripper_close,
        )
        self.t = 0
        self.arm = None
        self.button = 0
        self._gripper_tick = None

    def attach(self, arm):
        self.arm = arm
        self.arm.home_pose = self.init_qpos.copy()
        self.arm.Go_Home_Pose()
        self.last_sent = np.asarray(self.arm.target_pose).copy()
        self._gripper_tick = time.monotonic()

    def _simulate(self, q, viewer):
        self.mjdata.qpos[self.qpos_indices] = q[:6]
        finger = np.clip((q[6] - self.calibration.gripper_close) /
                         (self.calibration.gripper_open - self.calibration.gripper_close), 0., 1.)
        finger *= self.mjmodel.joint("finger_left").range[1]
        self.mjdata.qpos[self.finger_indices] = [finger, -finger]
        self.mjdata.ctrl[:] = np.r_[q[:6], finger]
        mujoco.mj_step(self.mjmodel, self.mjdata)
        if viewer is not None:
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.mjdata.time % 2)
            viewer.sync()

    def step(self, viewer=None):
        if self.arm is None:
            raise RuntimeError("Attach the SDK before running example 6")
        if self.t == 0:
            self._simulate(self.init_qpos, viewer)
        self.t += 1
        now = time.monotonic()
        dt = min(max(now - self._gripper_tick, 0.0), 0.1)
        self._gripper_tick = now
        target_gpos, gripper, self.button = self.joycon.get_control()
        target_gpos = np.asarray(target_gpos, dtype=float).copy()
        if target_gpos.shape != (6,) or not np.isfinite(target_gpos).all() or not np.isfinite(gripper):
            raise ValueError("Non-finite or invalid Joy-Con target")
        gripper = np.clip(gripper, *self.control_qlimit[:, 6])
        speed = abs(self.calibration.gripper_open - self.calibration.gripper_close) / GRIPPER_TRAVEL_TIME_S
        self.target_qpos[6] = self.last_sent[6] + np.clip(gripper - self.last_sent[6], -speed * dt, speed * dt)
        target_gpos[3:5] *= -1
        fd_qpos = np.clip(self.mjdata.qpos[self.qpos_indices], self.robot.qlim[0], self.robot.qlim[1])
        q, success = rebot_IK(fd_qpos, target_gpos, robot=self.robot)
        if success:
            self.target_qpos[:6] = q
        else:
            # Arm reachability must not pause the independent gripper ramp.
            # Hold the six axes at their last command.
            self.target_qpos[:6] = self.last_sent[:6]
        self.target_qpos = np.clip(self.target_qpos, self.control_qlimit[0], self.control_qlimit[1])
        if self.t < STARTUP_HOME_CYCLES:
            self.arm.Go_Home_Pose()
            self.last_sent = np.asarray(self.arm.target_pose).copy()
        elif self.t > STARTUP_HOME_CYCLES:
            self.arm.ReBot_Control(self.target_qpos)
            self.last_sent = np.asarray(self.arm.target_pose).copy()
        for _ in range(SIMULATION_STEPS):
            self._simulate(self.target_qpos, viewer)
        self.target_qpos_last = self.target_qpos.copy()
        return self.last_sent.copy(), self.button

    def close_joycon(self):
        self.joycon.running = False
        self.joycon.thread.join(timeout=2.)
        for name in ("joycon", "gyro", "button"):
            reader = getattr(self.joycon, name, None)
            if reader is not None:
                reader.enable = False
                reader._close()


def run_control_loop(controller, duration_s=1000, after_step=None, should_stop=None):
    """The standalone viewer/control loop, also used by LeRobot pure teleoperation."""
    import mujoco.viewer

    os.environ.setdefault("MUJOCO_GL", "egl")
    with mujoco.viewer.launch_passive(controller.mjmodel, controller.mjdata) as viewer:
        viewer.cam.azimuth, viewer.cam.elevation = 45, -30
        viewer.cam.distance, viewer.cam.lookat = 1., [-0.1, 0.2, 0.3]
        start = time.monotonic()
        while (not (should_stop is not None and should_stop()) and viewer.is_running()
               and (duration_s is None or time.monotonic() - start < duration_s)):
            tick = time.monotonic()
            sent, button = controller.step(viewer)
            if after_step is not None:
                after_step(sent, button)
            time.sleep(max(0., CONTROL_PERIOD_S - (time.monotonic() - tick)))


def main():
    os.environ.setdefault("MUJOCO_GL", "egl")
    np.set_printoptions(linewidth=200)
    arm_tty = select_port()
    controller = Example6Controller()
    arm = None
    normal_exit = False
    try:
        arm = ReBot(driver_port=arm_tty, arm_type="rebot_b601_dm", side="right")
        controller.attach(arm)
        run_control_loop(controller)
        normal_exit = True
    except KeyboardInterrupt:
        print("User interrupted the simulation.")
    finally:
        try:
            if arm is not None:
                try:
                    if normal_exit:
                        for _ in range(EXIT_HOME_CYCLES):
                            arm.Go_Home_Pose()
                finally:
                    try:
                        arm.ReBot_Disable()
                    finally:
                        arm.MotorController.serial_.close()
        finally:
            controller.close_joycon()


if __name__ == "__main__":
    main()
