
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from benchmark_runner import load_config, execute_batch
from trial_lifecycle import TrialLifecycle
from trial_metrics import TrialMetrics
from trial_monitor import TrialMonitor, TrialResult, TrialEvent

CONFIG = Path(__file__).resolve().parents[1] / 'config/benchmark.yaml'


class MonitorTests(unittest.TestCase):
  def setUp(self):
    self.config = load_config(CONFIG)
    self.monitor = TrialMonitor(self.config, 'balance_beam')
    self.monitor.armed_wall = 100

  def snapshot(self, sim, x=-3, y=0, z=0.575, roll=0):
    return {'clock': sim, 'clock_wall': 100 + sim, 'wall': 100 + sim,
            'state': {'stamp': sim, 'wall': 100 + sim,
                      'values': [roll, 0, 0, x, y, z] + [0] * 30},
            'motion_start': {'sim': 1, 'wall': 101}}

  def test_goal_requires_hold_and_measures_from_motion(self):
    self.assertIsNone(self.monitor.update(self.snapshot(1), 101))
    self.assertIsNone(self.monitor.update(self.snapshot(2, x=2), 102))
    result = self.monitor.update(self.snapshot(3, x=2), 103)
    self.assertEqual(result.status, 'success')
    self.assertEqual(result.time_to_goal_sim_sec, 2)

  def test_candidate_clears_without_recovery(self):
    self.monitor.update(self.snapshot(1, roll=1), 101)
    self.monitor.update(self.snapshot(1.2), 101.2)
    kinds = [e.kind for e in self.monitor.events]
    self.assertIn('failure_candidate_started', kinds)
    self.assertIn('failure_candidate_cleared', kinds)
    self.assertFalse(any('recovery' in kind for kind in kinds))

  def test_failure_precedes_success_and_timeout(self):
    self.config['success']['require_upright'] = False
    self.config['trial_timeout_sim_sec'] = 1
    self.monitor.update(self.snapshot(1, x=2, roll=1), 101)
    result = self.monitor.update(self.snapshot(2, x=2, roll=1), 102)
    self.assertEqual((result.status, result.reason), ('failure', 'fall'))

  def test_segment_boundary(self):
    self.monitor.update(self.snapshot(1, x=0, y=0.4), 101)
    result = self.monitor.update(self.snapshot(1.5, x=0, y=0.4), 101.5)
    self.assertEqual(result.reason, 'course_boundary')

  def test_stale_clock_regression_and_activation(self):
    snapshot = self.snapshot(1)
    snapshot['state']['wall'] = 90
    self.assertEqual(self.monitor.update(snapshot, 101).reason, 'stale_state')
    self.setUp()
    self.monitor.update(self.snapshot(1), 101)
    self.assertEqual(self.monitor.update(self.snapshot(0.5), 101.5).reason, 'clock_regression')
    self.setUp()
    snapshot = self.snapshot(31)
    snapshot['motion_start'] = None
    self.assertEqual(self.monitor.update(snapshot, 131).reason, 'activation_timeout')

  def test_stall_and_both_duration_caps(self):
    snapshot = self.snapshot(1)
    snapshot['clock_wall'] = 90
    self.assertEqual(self.monitor.update(snapshot, 101).reason, 'clock_stalled')
    self.setUp()
    self.config['trial_timeout_sim_sec'] = 1
    self.monitor.update(self.snapshot(1), 101)
    self.assertEqual(self.monitor.update(self.snapshot(2), 102).status, 'timeout')
    self.setUp()
    self.config['trial_timeout_wall_sec'] = 0.5
    self.monitor.update(self.snapshot(1), 101)
    self.assertEqual(self.monitor.update(self.snapshot(1.6), 101.6).reason, 'wall_duration_limit')

  def test_no_progress(self):
    self.monitor.update(self.snapshot(1), 101)
    result = self.monitor.update(self.snapshot(11), 111)
    self.assertEqual(result.reason, 'no_progress')


class MetricsTests(unittest.TestCase):
  def setUp(self):
    self.config = load_config(CONFIG)
    self.result = TrialResult('success', 'goal_reached', 'monitor', time_to_goal_sim_sec=1,
                             events=[TrialEvent('motion_start', '', 1), TrialEvent('termination', '', 2)])

  def summarize(self, samples):
    class Bag:
      def __init__(self, *args): pass
      def __enter__(self): return self
      def __exit__(self, *args): pass
      def read_messages(self, topics): return iter(samples)
    with patch.dict(sys.modules, rosbag=SimpleNamespace(Bag=Bag)):
      return TrialMetrics().summarize('recording.bag', self.result, self.config, 'balance_beam')

  def state(self, stamp, x):
    timestamp = SimpleNamespace(to_sec=lambda: stamp)
    return ('/anymal_d/state', SimpleNamespace(stamp=timestamp, rbd_state=[0, 0, 0, x, 0, 0.575] + [0] * 30), timestamp)

  def test_excludes_reset_and_cleanup_and_skips_duplicates(self):
    summary = self.summarize([self.state(0, -100), self.state(1, 0), self.state(1, 99),
                              self.state(1.5, 1), self.state(2, 2), self.state(3, 100)])
    self.assertEqual(self.result.distance_traveled_xy_m, 2)
    self.assertEqual(self.result.final_goal_distance_xy_m, 0)
    self.assertEqual(self.result.time_to_goal_sim_sec, 1)
    self.assertIsNone(summary['components']['mpc']['mean'])

  def test_gap_marks_distance_unavailable(self):
    summary = self.summarize([self.state(1, 0), self.state(2, 2)])
    self.assertIsNone(self.result.distance_traveled_xy_m)
    self.assertEqual(summary['trajectory']['reason'], 'state_gap')

  def test_runtime_units_and_invalid_values(self):
    self.config['runtime_sources']['mpc'] = {'topic': '/timing', 'field': 'duration',
                                            'unit': 's', 'timestamp_field': 'stamp'}
    timestamp = SimpleNamespace(to_sec=lambda: 1.5)
    samples = [('/timing', SimpleNamespace(stamp=timestamp, duration=value), timestamp)
               for value in (0.001, 0.003, float('nan'))]
    summary = self.summarize(samples)['components']['mpc']
    self.assertEqual(summary['mean'], 2)
    self.assertEqual(summary['invalid_samples'], 1)


class LifecycleTests(unittest.TestCase):
  def setUp(self):
    self.config = load_config(CONFIG)
    self.trial = TrialLifecycle(self.config, 'balance_beam', 'MPC', 1)

  def test_prepare_failure_cleans_up_and_returns_error(self):
    with patch.object(self.trial, 'prepare', side_effect=OSError('catkin missing')), \
         patch.object(self.trial, 'cleanup', return_value=[]) as cleanup:
      result = self.trial.execute()
    self.assertEqual(result.status, 'error')
    self.assertEqual(result.stage, 'prepare')
    cleanup.assert_called_once()

  def test_interrupt_and_cleanup_error_persist(self):
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp) / 'batch/world/MPC/trial_001'
      self.trial.attempt_dir.mkdir(parents=True)
      with patch.object(self.trial, 'prepare'), \
           patch.object(self.trial, 'launch_and_reset', side_effect=KeyboardInterrupt()), \
           patch.object(self.trial, 'cleanup', return_value=['remaining process']):
        result = self.trial.execute()
      self.assertEqual(result.status, 'interrupted')
      saved = json.loads((self.trial.attempt_dir / 'result.json').read_text())
      self.assertEqual(saved['cleanup_errors'], ['remaining process'])

  def test_analysis_failure_keeps_trial_outcome(self):
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp) / 'batch/world/MPC/trial_001'
      self.trial.attempt_dir.mkdir(parents=True)
      (self.trial.attempt_dir / 'recording.bag').touch()
      outcome = TrialResult('failure', 'fall', 'monitor')
      with patch.object(self.trial, 'prepare'), patch.object(self.trial, 'launch_and_reset'), \
           patch.object(self.trial, 'record_and_run', return_value=outcome), \
           patch.object(self.trial, 'cleanup', return_value=[]), \
           patch.object(TrialMetrics, 'summarize', side_effect=ValueError('bad bag')):
        result = self.trial.execute()
      self.assertEqual(result.reason, 'fall')
      self.assertTrue((self.trial.attempt_dir / 'recording.bag').exists())
      self.assertEqual(json.loads((self.trial.attempt_dir / 'runtime_summary.json').read_text()),
                       {'analysis_error': 'bad bag'})

  def test_launch_reset_and_mapping_order(self):
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp)
      self.trial.env = {'ROS_MASTER_URI': 'http://127.0.0.1:23456'}
      order = []
      def start(name, command):
        order.append(name)
        return SimpleNamespace(poll=lambda: 0, returncode=0)
      def wait(condition, timeout, reason, allow_exit=()):
        now = time.monotonic() + 0.01
        snapshot = {'wall': now, 'clock': 1, 'clock_wall': now,
                    'state': {'wall': now, 'values': [0, 0, 0, -3, 0, 0.575] + [0] * 30},
                    'services': ['/gazebo/pause_physics', '/gazebo/unpause_physics',
                                 '/gazebo/set_model_state', '/gazebo/set_model_configuration']}
        if reason == 'reset_verification_timeout':
          self.assertFalse(condition(snapshot))
          snapshot['clock'] = 2
        self.assertTrue(condition(snapshot), reason)
        return snapshot
      with patch.object(self.trial, '_start', side_effect=start), patch.object(self.trial, '_wait', side_effect=wait):
        self.trial.launch_and_reset()
      self.assertEqual(order, ['master', 'observer', 'launch', 'reset', 'mapping'])

  def test_reset_failure_does_not_start_mapping(self):
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp)
      self.trial.env = {'ROS_MASTER_URI': 'http://127.0.0.1:23456'}
      with patch.object(self.trial, '_start', return_value=SimpleNamespace(returncode=1)) as start, \
           patch.object(self.trial, '_wait'):
        with self.assertRaisesRegex(RuntimeError, 'reset_failed'):
          self.trial.launch_and_reset()
      self.assertNotIn('mapping', [call.args[0] for call in start.call_args_list])

  def test_rl_and_dtc_wait_for_their_map_before_activation(self):
    for baseline, topic in [('RL', '/elevation_mapping/elevation_map_raw'),
                            ('DTC', '/convex_plane_decomposition_ros/filtered_map')]:
      with self.subTest(baseline=baseline), tempfile.TemporaryDirectory() as tmp:
        trial = TrialLifecycle(self.config, 'balance_beam', baseline, 1)
        trial.attempt_dir = Path(tmp)
        trial.env = {'ROS_MASTER_URI': 'http://127.0.0.1:23456'}
        probe_codes = iter([None, 0])
        probe = SimpleNamespace(poll=lambda: next(probe_codes), returncode=0)
        def start(name, command):
          return probe if name == 'mapping_ready' else SimpleNamespace(returncode=0)
        def wait(condition, timeout, reason, allow_exit=()):
          if reason == 'mapping_timeout':
            self.assertFalse(condition({}))
            self.assertTrue(condition({}))
            self.assertEqual(timeout, self.config['readiness']['mapping_timeout_wall_sec'])
        with patch.object(trial, '_start', side_effect=start) as launch, \
             patch.object(trial, '_wait', side_effect=wait):
          trial.launch_and_reset()
        self.assertEqual(launch.call_args.args,
                         ('mapping_ready', ['rostopic', 'echo', '-n', '1', '--noarr', topic]))
        self.assertIn('mapping_ready', trial.completed_processes)
        trial.processes['mapping_ready'] = SimpleNamespace(poll=lambda: 0)
        with patch.object(trial, '_track'):
          trial._health()  # Successful one-shot exit is not a controller crash.

  def test_mapping_timeout_stops_before_run(self):
    with tempfile.TemporaryDirectory() as tmp:
      trial = TrialLifecycle(self.config, 'balance_beam', 'RL', 1)
      trial.attempt_dir = Path(tmp) / 'batch/balance_beam/RL/trial_001'
      trial.attempt_dir.mkdir(parents=True)
      trial.env = {'ROS_MASTER_URI': 'http://127.0.0.1:23456'}
      def wait(condition, timeout, reason, allow_exit=()):
        if reason == 'mapping_timeout':
          raise RuntimeError(reason)
      with patch.object(trial, 'prepare'), patch.object(trial, '_wait', side_effect=wait), \
           patch.object(trial, '_start', return_value=SimpleNamespace(returncode=0)), \
           patch.object(trial, 'cleanup', return_value=[]), patch.object(trial, 'record_and_run') as run:
        result = trial.execute()
      run.assert_not_called()
      self.assertEqual((result.status, result.stage, result.reason), ('error', 'mapping', 'mapping_timeout'))

  def test_recorder_and_arm_before_run(self):
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp)
      order = []
      def start(name, command):
        order.append(name)
        if name == 'run':
          self.assertTrue((self.trial.attempt_dir / 'arm').exists())
      def wait(condition, timeout, reason, allow_exit=()):
        self.assertEqual(reason, 'monitor_arm_timeout')
        self.assertTrue(condition({'armed': True}))
      with patch.object(self.trial, '_start', side_effect=start), \
           patch.object(self.trial, '_wait', side_effect=wait), \
           patch.object(TrialMonitor, 'wait_for_result', return_value=TrialResult('success', '', 'monitor')):
        self.assertEqual(self.trial.record_and_run().status, 'success')
      self.assertEqual(order, ['record', 'run'])

  def test_batch_stops_on_cleanup_error(self):
    bad = TrialResult('success', 'goal_reached', 'monitor', cleanup_errors=['recording unfinished'])
    with patch('benchmark_runner.TrialLifecycle.execute', return_value=bad) as execute:
      self.assertEqual(len(execute_batch(self.config)), 1)
      execute.assert_called_once()

  def test_stop_detached_descendant_and_preserve_unrelated_process(self):
    import psutil
    with tempfile.TemporaryDirectory() as tmp:
      self.trial.attempt_dir = Path(tmp)
      self.trial.env = dict(os.environ, TBAI_BENCHMARK_ATTEMPT=tmp)
      self.trial.subprocess_kwargs = {'cwd': tmp, 'env': self.trial.env, 'start_new_session': True}
      self.config['cleanup'].update(process_sigint_timeout_wall_sec=0.15,
                                    process_sigterm_timeout_wall_sec=0.15,
                                    process_sigkill_timeout_wall_sec=1)
      child_code = 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'
      parent_code = ('import subprocess,sys,time; from pathlib import Path; '
                     f'p=subprocess.Popen([sys.executable,"-c",{child_code!r}],start_new_session=True); '
                     'Path("child.pid").write_text(str(p.pid)); time.sleep(60)')
      unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
      child_pid = None
      try:
        self.trial._start('run', [sys.executable, '-c', parent_code])
        deadline = time.monotonic() + 3
        while not (Path(tmp) / 'child.pid').exists() and time.monotonic() < deadline:
          time.sleep(0.02)
        child_pid = int((Path(tmp) / 'child.pid').read_text())
        time.sleep(0.1)
        self.trial._stop('run')
        self.assertIsNone(unrelated.poll())
        self.assertTrue(not psutil.pid_exists(child_pid) or psutil.Process(child_pid).status() == psutil.STATUS_ZOMBIE)
      finally:
        unrelated.terminate()
        unrelated.wait()
        self.trial._stop('run')
        for log in self.trial.process_logs.values(): log.close()


if __name__ == '__main__':
  unittest.main()
