"""Shared feedback-triggered position protection; no hardware or GUI imports."""

import math

GRIPPER_TORQUE_TRIP_NM = 1.0


class GripperTorqueProtection:
  def __init__(self, record_event, port: str):
    self.record_event = record_event
    self.driver_port = port
    self.hold: float | None = None
    self.blocked_direction = 0.0

  def apply(self, requested: float, position: float, measured: float) -> float:
    """Clamp further travel after overload; allow an explicit reverse target."""
    if not all(math.isfinite(value) for value in (requested, position, measured)):
      raise ValueError("Gripper target/feedback must be finite")
    overloaded = abs(measured) > GRIPPER_TORQUE_TRIP_NM
    if self.hold is not None:
      reversing = ((requested - self.hold)
                   * self.blocked_direction < -1e-6)
      if reversing:
        # Permit unloading even while feedback is still above the threshold.
        # Keep the latch until unloaded, so the next frame cannot retrigger
        # protection against the escape direction.
        if not overloaded:
          self.hold = None
          self.blocked_direction = 0.0
          self.record_event("gripper_torque_release", port=self.driver_port,
                       requested_target=requested, feedback_torque=measured)
        return requested
    if overloaded:
      if self.hold is None:
        error = requested - position
        self.blocked_direction = float(math.copysign(1.0, error if abs(error) > 1e-6 else measured))
        self.hold = position
      else:
        # Only move the boundary back, never creep farther into the obstacle.
        if ((position - self.hold)
            * self.blocked_direction < 0):
          self.hold = position
    if self.hold is not None:
      self.record_event("gripper_torque_protection", port=self.driver_port,
                   threshold_nm=GRIPPER_TORQUE_TRIP_NM,
                   feedback_torque=measured, requested_target=requested,
                   protected_target=float(self.hold),
                   blocked_direction=self.blocked_direction)

    return requested if self.hold is None else self.hold
