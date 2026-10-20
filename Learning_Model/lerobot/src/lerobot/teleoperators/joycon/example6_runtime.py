"""Shared example 6 control with snapshots and a separate recording/sampling thread."""

import threading
import time
from collections.abc import Callable
from types import ModuleType
from typing import Any

import numpy as np

from lerobot.robots.rebot_b601_follower.motor_family import JOINT_NAMES


def positions(values: Any) -> dict[str, float]:
    return dict(zip((f"{name}.pos" for name in JOINT_NAMES), np.rad2deg(values).tolist(), strict=True))


class Example6Runtime:
    def __init__(self, module: ModuleType, controller: Any, robot: Any):
        self.module, self.controller, self.robot = module, controller, robot
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._error: BaseException | None = None
        self._button = 0
        self._events: list[int] = []
        self._action = positions(controller.init_qpos)
        self._observation = dict(self._action)
        self.thread = threading.Thread(target=self._run, name="rebot-example6", daemon=True)

    def start(self, *, background: bool = True) -> None:
        with self.robot._example_lock:
            self.controller.attach(self.robot._example_sdk)
            self._observation = positions(self.robot._example_sdk.motors_pose)
            self._action = positions(self.controller.last_sent)
        if background:
            self.thread.start()

    def _publish(self, sent: Any, button: int) -> None:
        measured = np.asarray(self.robot._example_sdk.motors_pose).copy()
        if not np.isfinite(measured).all():
            raise RuntimeError("Non-finite SDK feedback")
        with self._lock:
            self._action, self._observation = positions(sent), positions(measured)
            if button in (1, -1) and button != self._button:
                self._events.append(button)
            self._button = button

    def run_foreground(
        self,
        duration_s: float | None,
        on_step: Callable[[], None] | None = None,
        should_stop: Callable[[], bool] | None = None,
    ) -> None:
        """Use exactly the standalone main-thread viewer loop; no worker or outer FPS pacing."""
        if self.thread.ident is not None:
            raise RuntimeError("Cannot run foreground control alongside the worker")

        def after_step(sent: Any, button: int) -> None:
            self._publish(sent, button)
            if on_step is not None:
                on_step()

        try:
            self.module.run_control_loop(
                self.controller, duration_s=duration_s, after_step=after_step,
                should_stop=should_stop or self._stop.is_set,
            )
        except BaseException as exc:
            self._error = exc
            self._stop.set()
            with self.robot._example_lock:
                self.robot._example_sdk.ReBot_Disable()
            raise

    def run_recording(self, record_session: Callable[[], None], cancel: Callable[[], None]) -> None:
        """Keep the shared viewer/control loop on the caller; sample/save on a separate thread."""
        done = threading.Event()
        errors: list[BaseException] = []

        def record() -> None:
            try:
                record_session()
            except BaseException as exc:
                errors.append(exc)
            finally:
                done.set()

        sampler = threading.Thread(target=record, name="rebot-recording")

        def start_sampling() -> None:
            # Wait for a real control snapshot and a working viewer before recording.
            if sampler.ident is None:
                sampler.start()

        try:
            self.run_foreground(None, on_step=start_sampling, should_stop=done.is_set)
            if errors:
                raise errors[0]
            if not done.is_set():
                raise InterruptedError("MuJoCo window closed before recording finished")
        except BaseException as exc:
            if self._error is None:
                self._error = exc
                self._stop.set()
                with self.robot._example_lock:
                    self.robot._example_sdk.ReBot_Disable()
            raise
        finally:
            # Window close / Ctrl+C / control failure must stop every recording phase.
            # Join before the caller finalizes the dataset or disconnects cameras.
            cancel()
            if sampler.ident is not None:
                sampler.join()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                tick = time.monotonic()
                with self.robot._example_lock:
                    sent, button = self.controller.step()
                    self._publish(sent, button)
                self._stop.wait(max(0.0, self.module.CONTROL_PERIOD_S - (time.monotonic() - tick)))
        except BaseException as exc:
            self._error = exc
            self._stop.set()
            with self.robot._example_lock:
                self.robot._example_sdk.ReBot_Disable()

    def snapshot(self) -> tuple[dict[str, float], dict[str, float]]:
        if self._error is not None:
            raise RuntimeError("Example 6 control loop failed") from self._error
        with self._lock:
            return dict(self._observation), dict(self._action)

    def consume_buttons(self) -> list[int]:
        with self._lock:
            buttons, self._events = self._events, []
            return buttons

    def stop(self) -> None:
        self._stop.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                raise RuntimeError("Example 6 worker did not stop; refusing concurrent SDK access")
