"""Regressions for Go2's measured SIT offset and recording startup failures."""

from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from trial_lifecycle import TrialLifecycle


class StartupReadinessTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "robot_readiness": {
                "state_stale_wall_sec": 1.0,
                "sit_joint_tolerance_rad": 0.15,
            },
            "monitor": {"state_topic": "/anymal_d/state"},
            "record_topics": ["/anymal_d/state", "/elevation_mapping/elevation_map_raw"],
        }
        self.trial = TrialLifecycle(self.config, "balance_beam", "MPC", 1)
        self.trial.sit_joint_positions = [0.0, 1.36, -2.55] * 4
        # Final settled pose from the failing Go2 startup recording. The rear
        # knees have steady offsets while all joint speeds are below 0.003 rad/s.
        self.values = [0.0] * 12 + [
            0.028681, 1.365057, -2.650255,
            0.032032, 1.346821, -2.667999,
            -0.029935, 1.367945, -2.649719,
            -0.030641, 1.342617, -2.668804,
        ] + [0.002] * 12

    def sample(self, stamp):
        now = time.monotonic()
        self.trial.latest = (stamp, tuple(self.values), now)
        self.trial.last_state_advance = now
        return self.trial.sitting()

    def test_settled_go2_pose_passes_after_hold(self):
        self.assertFalse(self.sample(10.0))
        self.assertFalse(self.sample(10.1))
        self.assertTrue(self.sample(10.25))

    def test_default_tolerance_reproduces_measured_failure(self):
        del self.config["robot_readiness"]["sit_joint_tolerance_rad"]
        self.assertFalse(self.sample(10.0))
        self.assertFalse(self.sample(11.0))

    def test_motion_restarts_hold(self):
        self.assertFalse(self.sample(10.0))
        self.values[24] = 0.3
        self.assertFalse(self.sample(10.25))
        self.values[24] = 0.002
        self.assertFalse(self.sample(10.5))
        self.assertTrue(self.sample(10.75))

    def test_excess_position_error_restarts_hold(self):
        self.assertFalse(self.sample(10.0))
        self.values[12] = 0.16
        self.assertFalse(self.sample(10.25))
        self.values[12] = 0.028681
        self.assertFalse(self.sample(10.5))
        self.assertTrue(self.sample(10.75))

    def test_stale_state_cannot_pass(self):
        self.assertFalse(self.sample(10.0))
        old = time.monotonic() - 2.0
        self.trial.latest = (10.25, tuple(self.values), old)
        self.trial.last_state_advance = old
        self.assertFalse(self.trial.sitting())
        self.assertIsNone(self.trial.sitting_since)

    def test_recording_does_not_wait_for_mapping_to_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.trial.attempt_dir = Path(tmp)
            subscriptions = [["/anymal_d/state", ["/benchmark_recorder"]]]
            with patch("trial_lifecycle.rosgraph.Master") as master:
                master.return_value.getSystemState.return_value = ([], subscriptions, [])
                self.assertFalse(self.trial.recording_ready())
                (Path(tmp) / "recording.bag.active").touch()
                self.assertTrue(self.trial.recording_ready())
                subscriptions.clear()
                self.assertFalse(self.trial.recording_ready())


if __name__ == "__main__":
    unittest.main()
