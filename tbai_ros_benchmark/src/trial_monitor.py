"""Evaluate trial outcomes from the ROS observer's snapshots."""

from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import List, Optional


@dataclass
class TrialEvent:
  kind: str
  reason: str
  sim_time_sec: Optional[float] = None
  wall_elapsed_sec: Optional[float] = None
  recovery_id: Optional[str] = None
  recovery_outcome: Optional[str] = None


@dataclass
class TrialResult:
  status: str
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
  def __init__(self, config, world):
    self.config, self.world = config, world
    self.settings = config['world_settings'][world]
    self.armed_wall = time.monotonic()
    self.start_sim = self.start_wall = self.last_sim = None
    self.state_stamp = None
    self.events = []
    self.candidates = {}
    self.progress = deque()
    self.best_distance = math.inf
    self.result = None

  def event(self, kind, reason, sim, wall):
    self.events.append(TrialEvent(kind, reason, sim, wall - self.armed_wall))

  def finish(self, status, reason, sim, wall, stage='monitor'):
    if self.result is None:
      duration = None if self.start_sim is None or sim is None else max(0, sim - self.start_sim)
      elapsed = None if self.start_wall is None else wall - self.start_wall
      self.event('termination', reason, sim, wall)
      self.result = TrialResult(status, reason, stage, duration, elapsed,
                                duration if status == 'success' else None, events=self.events)
    return self.result

  def held(self, name, active, hold, sim, wall):
    if active:
      if name not in self.candidates:
        self.candidates[name] = sim
        if name != 'goal' and self.config['monitor']['record_intermediate_events']:
          self.event('failure_candidate_started', name, sim, wall)
      return sim - self.candidates[name] >= hold
    if name in self.candidates:
      del self.candidates[name]
      if name != 'goal' and self.config['monitor']['record_intermediate_events']:
        self.event('failure_candidate_cleared', name, sim, wall)
    return False

  def observe_state(self, state, sim, wall):
    """Use the repository's world-frame RbdState layout and sustained criteria."""
    values = state['values']
    if len(values) != 36 or not all(math.isfinite(v) for v in values):
      return self.finish('error', 'invalid_state', sim, wall)
    stamp = state['stamp']
    if self.state_stamp is not None and stamp < self.state_stamp:
      return self.finish('error', 'state_time_regression', sim, wall)
    if stamp == self.state_stamp:
      return None
    self.state_stamp = stamp
    roll, pitch, _, x, y, z = values[:6]
    gx, gy = self.settings['goal_position'][:2]
    distance = math.hypot(x - gx, y - gy)
    success = self.config['success']
    near = distance <= success['xy_tolerance_m']
    upright = (abs(roll) <= success['max_abs_roll_rad'] and
               abs(pitch) <= success['max_abs_pitch_rad'])
    failure = self.settings['failure']
    fall = failure['fall']
    fallen = (z < fall['min_base_height_world_m'] or abs(roll) > fall['max_abs_roll_rad']
              or abs(pitch) > fall['max_abs_pitch_rad'])
    boundary = failure['course_boundary']
    outside = not (boundary['x_bounds_m'][0] <= x <= boundary['x_bounds_m'][1]
                   and boundary['y_bounds_m'][0] <= y <= boundary['y_bounds_m'][1])
    outside |= any(segment['x_bounds_m'][0] <= x <= segment['x_bounds_m'][1] and
                   not segment['y_bounds_m'][0] <= y <= segment['y_bounds_m'][1]
                   for segment in boundary.get('segments', []))
    self.best_distance = min(self.best_distance, distance)
    self.progress.append((sim, self.best_distance))
    progress = failure['no_progress']
    cutoff = sim - progress['window_sim_sec']
    while len(self.progress) > 1 and self.progress[1][0] <= cutoff:
      self.progress.popleft()
    stuck = (sim - self.start_sim >= progress['grace_sim_sec'] and
             self.progress[0][0] <= cutoff and
             self.progress[0][1] - self.best_distance < progress['min_progress_m'] and
             not (near and progress['exclude_goal_tolerance']))
    failed = []
    for name, active, hold in [('fall', fallen, fall['hold_sim_sec']),
                                ('course_boundary', outside, boundary['hold_sim_sec']),
                                ('no_progress', stuck, 0)]:
      if self.held(name, active, hold, sim, wall):
        failed.append(name)
    reached = self.held('goal', near and (upright or not success['require_upright']),
                        success['hold_sim_sec'], sim, wall)
    if failed:
      return self.finish('failure', failed[0], sim, wall)
    if reached:
      return self.finish('success', 'goal_reached', sim, wall)
    return None

  def update(self, snapshot, wall=None):
    wall = time.monotonic() if wall is None else wall
    sim = snapshot.get('clock')
    ready = self.config['readiness']
    state = snapshot.get('state')
    reason = None
    if sim is None or wall - snapshot.get('clock_wall', 0) > ready['clock_stall_wall_sec']:
      reason = 'clock_stalled'
    elif self.last_sim is not None and sim < self.last_sim:
      reason = 'clock_regression'
    elif state is None or wall - state['wall'] > ready['state_stale_wall_sec']:
      reason = 'stale_state'
    if reason:
      return self.finish('error', reason, sim, wall)
    self.last_sim = sim
    motion = snapshot.get('motion_start')
    if self.start_sim is None and motion is not None:
      self.start_sim, self.start_wall = motion['sim'], motion['wall']
      self.event('motion_start', 'first_nonzero_cmd_vel_after_run_script', self.start_sim, self.start_wall)
    if self.start_sim is None:
      if wall - self.armed_wall >= self.config['activation_timeout_wall_sec']:
        return self.finish('error', 'activation_timeout', sim, wall, 'run_script')
      return None
    outcome = self.observe_state(state, sim, wall)
    if outcome:
      return outcome
    if sim - self.start_sim >= self.config['trial_timeout_sim_sec']:
      return self.finish('timeout', 'simulation_duration_limit', sim, wall)
    if wall - self.start_wall >= self.config['trial_timeout_wall_sec']:
      return self.finish('timeout', 'wall_duration_limit', sim, wall)
    return None

  def wait_for_result(self, read_snapshot, check_health):
    while self.result is None:
      check_health()
      self.update(read_snapshot())
      if self.result is None:
        time.sleep(self.config['readiness']['poll_wall_sec'])
    return self.result
