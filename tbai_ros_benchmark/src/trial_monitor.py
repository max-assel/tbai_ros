"""ROS-independent result contract and placeholder for the ROS trial monitor."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class TrialResult:
  """One attempt; infrastructure errors must remain distinct from robot failures."""

  status: str  # success | failure | timeout | error | interrupted
  reason: str
  sim_duration_sec: Optional[float] = None
  wall_duration_sec: Optional[float] = None
  final_goal_distance_m: Optional[float] = None


class TrialMonitor:
  """Implement subscriptions and terminal conditions here, not by parsing logs."""

  def __init__(self, config, world):
    self.config = config
    self.world = world

  def wait_for_result(self) -> TrialResult:
    """Return exactly one terminal result using bounded monitoring."""
    # TODO: subscribe to /anymal_d/state and /clock; verify state frame and orientation convention.
    # TODO: start timing at the first motion command after controller activation.
    # TODO: require goal tolerance + upright posture for hold_sim_sec.
    # TODO: implement world-specific sustained fall, course exit and no-progress rules.
    # TODO: use simulation time for trial duration; time.monotonic() for watchdogs.
    # TODO: detect stale state, stalled clock, and required-node crashes as errors.
    # TODO: avoid rospy.Rate sleeps that can block forever when simulation time stops.
    raise NotImplementedError("Implement trial monitoring and terminal conditions")
