"""Skeleton for duration-limited trials with early outcomes and intermediate events."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class TrialEvent:
  """A timestamped event, optionally belonging to a recovery episode."""
  kind: str  # motion_start | intermediate_failure | recovery_start | recovery_end | termination
  reason: str
  sim_time_sec: Optional[float] = None
  wall_elapsed_sec: Optional[float] = None
  recovery_id: Optional[str] = None
  recovery_outcome: Optional[str] = None  # succeeded | failed | incomplete | unknown


@dataclass
class TrialResult:
  """Final outcome stays separate from recoverable events and cleanup errors."""
  status: str  # success | failure | timeout | error | interrupted
  reason: str
  stage: str
  sim_duration_sec: Optional[float] = None
  wall_duration_sec: Optional[float] = None
  time_to_goal_sim_sec: Optional[float] = None
  distance_traveled_xy_m: Optional[float] = None
  final_goal_distance_xy_m: Optional[float] = None
  events: List[TrialEvent] = field(default_factory=list)
  cleanup_errors: List[str] = field(default_factory=list)


class TrialMonitor:
  """Observe the existing controller; do not add a recovery controller."""

  def __init__(self, config, world):
    self.config, self.world = config, world

  def observe_state(self, message):
    """Evaluate configured criteria using verified, fresh state."""
    # TODO: Verify message layout, orientation convention, frame, timestamp and validity.
    # Suggestion: transform pose to goal frame; use sustained/debounced goal, fall,
    # boundary and no-progress checks, resetting hold timers when data becomes stale.
    # TODO: Distinguish intermediate recoverable events from terminal failures.
    # Suggestion: terminal is the default for configured failure rules; explicitly define
    # which episodes may recover and their grace/attempt limits using replayed bags.
    raise NotImplementedError("State-based success/failure checks")

  def observe_event(self, event):
    """Record intermediate failures and recovery supplied by existing controller logic."""
    # TODO: Consume configured event stream; assign IDs and pair recovery start/end events.
    # Suggestion: log one event per episode, not per state sample; controller switch alone
    # does not prove recovery. No source means unavailable recovery metrics, not zero.
    # TODO: Preserve successful, failed and incomplete recoveries independently of final outcome.
    # Suggestion: successful trials may include recoveries; leave controller behavior unchanged.
    raise NotImplementedError("Intermediate events and recovery history")

  def wait_for_result(self) -> TrialResult:
    """End at first qualifying outcome or maximum duration."""
    # TODO: Arm before run script; mark motion_start after controller activation.
    # Suggestion: use activation_timeout_wall_sec to catch scripts that never begin motion.
    # TODO: End on confirmed success, terminal failure or trial_timeout_sim_sec.
    # Suggestion: simulation time measures travel; recovery never restarts the duration clock.
    # Define tie precedence explicitly, e.g. infrastructure error, failure, success, timeout.
    # TODO: Watch stale state/clock, process crashes and recorder failure with wall deadlines.
    # Suggestion: poll using monotonic wall time, not sleeps dependent on advancing ROS time.
    # Report infrastructure issues as error, duration exhaustion as timeout, user stop as interrupted.
    # TODO: Return TrialResult and events, including incomplete recoveries.
    # Suggestion: time_to_goal is null unless goal completion is confirmed; summarize trajectory offline.
    raise NotImplementedError("Duration cap plus early success/failure")
