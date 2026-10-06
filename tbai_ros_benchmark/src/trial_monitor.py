"""Outcome detection using robot-state samples after motion activation."""
from dataclasses import dataclass, field
import math


@dataclass
class TrialResult:
    status: str
    reason: str
    duration_sim_sec: float = 0.0
    recovery_events: list = field(default_factory=list)
    cleanup_errors: list = field(default_factory=list)
    forward_progress_m: float = 0.0


class TrialMonitor:
    def __init__(self, config, world):
        settings = config['world_settings'][world]
        self.start, self.goal = settings['start_position'], settings['goal_position']
        self.success = config['success']
        self.recovery = {**config['recovery'], **settings.get('recovery', {})}
        self.fall = settings['failure']['fall']
        self.boundary = settings['failure']['course_boundary']
        self.no_progress = settings['failure']['no_progress']
        self.timeout = config['timeout_sim_sec']
        dx, dy = self.goal[0] - self.start[0], self.goal[1] - self.start[1]
        self.route = (dx, dy, math.hypot(dx, dy))
        self.started = None
        self.last_stamp = None
        self.holds = {}
        self.ref = None
        self.events = []
        self.disturbance = None
        self.duration = 0.0
        self.terrain_entered = False
        self.progress = 0.0

    def held(self, name, condition, stamp, seconds):
        if not condition:
            self.holds.pop(name, None)
            return False
        self.holds.setdefault(name, stamp)
        return stamp - self.holds[name] >= seconds

    def result(self, status, reason):
        return TrialResult(status, reason, self.duration, list(self.events),
                           forward_progress_m=self.progress)

    def on_terrain(self, x, y):
        # Footprints are convex polygons with counterclockwise vertices.
        return any(all((b[0] - a[0]) * (y - a[1])
                       - (b[1] - a[1]) * (x - a[0]) >= -1e-9
                       for a, b in zip(polygon, polygon[1:] + polygon[:1]))
                   for polygon in self.boundary['polygons_m'])

    def update(self, state):
        stamp, values, _ = state
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                return self.result('error', 'simulation_time_reversed')
            if stamp == self.last_stamp:
                return None
        self.last_stamp = stamp
        if self.started is None:
            self.started = stamp
        self.duration = stamp - self.started
        roll, pitch = values[:2]
        x, y, z = values[3:6]
        start, goal = self.start, self.goal
        dx, dy, length = self.route
        progress = ((x - start[0]) * dx + (y - start[1]) * dy) / length
        self.progress = max(0.0, min(length, progress))
        success, fall = self.success, self.fall
        upright = (abs(roll) <= success['max_abs_roll_rad']
                   and abs(pitch) <= success['max_abs_pitch_rad'])
        at_goal = math.hypot(x - goal[0], y - goal[1]) <= success['xy_tolerance_m']
        boundary = self.boundary
        self.terrain_entered |= ((x - boundary['terrain_entry_x_m'])
                                 * self.route[0] >= 0)
        outside = self.terrain_entered and not self.on_terrain(x, y)
        if self.held('boundary', outside, stamp, boundary['hold_sim_sec']):
            return self.result('failure', 'fell_off_terrain')
        if self.held('tip', abs(roll) > fall['max_abs_roll_rad'] or
                     abs(pitch) > fall['max_abs_pitch_rad'], stamp, fall['hold_sim_sec']) and not outside:
            return self.result('failure', 'tipped_over')
        if self.held('low', z < fall['min_base_height_world_m'], stamp, fall['hold_sim_sec']) and not outside:
            return self.result('failure', 'base_below_height_limit')

        recovery = self.recovery
        disturbed = (abs(roll) > recovery['disturbance_roll_rad'] or
                     abs(pitch) > recovery['disturbance_pitch_rad'])
        if disturbed and self.disturbance is None:
            self.disturbance = (stamp, progress)
        recovered = (self.disturbance is not None and upright and not disturbed
                     and progress - self.disturbance[1] >= recovery['min_progress_m'])
        if self.held('recovery', recovered, stamp, recovery['hold_sim_sec']):
            self.events.append({'start_sim_sec': self.disturbance[0],
                                'end_sim_sec': stamp, 'kind': 'posture_recovery_estimate'})
            self.disturbance = None
            self.holds.pop('recovery', None)
        if self.held('goal', at_goal and upright,
                     stamp, success['hold_sim_sec']):
            return self.result('success', 'goal_reached')
        no_progress = self.no_progress
        if self.ref is None or progress - self.ref[1] >= no_progress['min_progress_m']:
            self.ref = (stamp, progress)
        elif (self.duration >= no_progress['grace_sim_sec']
              and stamp - self.ref[0] >= no_progress['window_sim_sec']):
            return self.result('timeout', 'stuck')
        if self.duration >= self.timeout:
            return self.result('timeout', 'duration_limit')
        return None
