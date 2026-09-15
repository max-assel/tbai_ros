"""Run one benchmark attempt using the existing robot scripts."""

from datetime import datetime, timezone
import os
import json
import math
import signal
import sys
import time
from urllib.parse import urlparse
from pathlib import Path
import socket
import subprocess
import tempfile

import yaml

BASELINES = {"MPC": "tbai_ros_mpc", "RL": "tbai_ros_bob", "DTC": "tbai_ros_dtc"}
STAGES = ("launch", "readiness", "reset_script", "verify_reset", "mapping", "record",
          "arm_monitor", "run_script", "monitor", "cleanup", "summarize")


class TrialLifecycle:
  """Own the processes and artifacts of one attempt."""

  def __init__(self, config, world, baseline, repetition):
    self.config, self.world, self.baseline = config, world, baseline
    self.repetition = repetition
    self.processes = {}
    self.process_logs = {}
    self.process_descendants = {}
    self.stage = "prepare"
    self.monitor = None

  def command_plan(self):
    """Return argument lists for preview and execution."""
    return {
      "launch": ["roslaunch", BASELINES[self.baseline], "anymal_d_perceptive.launch",
                 f"world:={self.world}", f"gui:={str(self.config['gazebo_gui']).lower()}",
                 f"rviz:={str(self.config.get('rviz', True)).lower()}"],
      "reset": ["bash", "-e", self.config["scripts"]["reset"], self.world, self.baseline],
      "mapping": ["roslaunch", "tbai_ros_gridmap", "elevation_mapping.launch"],
      "record": ["rosbag", "record", "-O",
                 str(self.attempt_dir / "recording.bag") if hasattr(self, "attempt_dir")
                 else "<attempt-dir>/recording.bag", *dict.fromkeys(
                   self.config["record_topics"] + [source['topic'] for source in
                     self.config.get('runtime_sources', {}).values() if source])],
      "run": ["bash", "-e", self.config["scripts"]["run"], self.world, self.baseline],
    }

  def prepare(self):
    """Set up one attempt, assuming a sourced workspace and valid configuration."""
    if hasattr(self, "attempt_dir"):
      raise ValueError("prepare may only be called once per attempt")
    self.env = os.environ.copy()
    self.workspace = Path(self._probe(["catkin", "locate"], Path.cwd())).resolve()

    config_path = Path(self.config.get("_config_path",
      Path(__file__).resolve().parents[1] / "config/benchmark.yaml"))
    for name in ("reset", "run"):
      script = (config_path.parent / self.config["scripts"][name]).resolve()
      self.config["scripts"][name] = str(script)

    output = (config_path.parent / self.config["output_dir"]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    # The shared config holds the batch path for subsequent sequential attempts.
    if "_batch_dir" not in self.config:
      stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
      self.config["_batch_dir"] = tempfile.mkdtemp(prefix=f"batch_{stamp}_", dir=output)
    self.batch_dir = Path(self.config["_batch_dir"])
    parent = self.batch_dir / self.world / self.baseline
    parent.mkdir(parents=True, exist_ok=True)
    self.attempt_dir = Path(tempfile.mkdtemp(prefix=f"trial_{self.repetition:03d}_", dir=parent))
    (self.attempt_dir / "ros_logs").mkdir()
    (self.attempt_dir / "ros_home").mkdir()
    # Bind both simultaneously so the OS selects distinct available ports.
    # Launch must still detect bind failures: ports cannot be reserved across exec.
    with socket.socket() as ros_socket, socket.socket() as gazebo_socket:
      ros_socket.bind(("127.0.0.1", 0))
      gazebo_socket.bind(("127.0.0.1", 0))
      self.env.update(
        ROS_MASTER_URI=f"http://127.0.0.1:{ros_socket.getsockname()[1]}",
        GAZEBO_MASTER_URI=f"http://127.0.0.1:{gazebo_socket.getsockname()[1]}",
        ROS_IP="127.0.0.1", ROS_HOME=str(self.attempt_dir / "ros_home"),
        ROS_LOG_DIR=str(self.attempt_dir / "ros_logs"),
      )
    self.env["TBAI_BENCHMARK_ATTEMPT"] = str(self.attempt_dir)
    self.env.pop("ROS_HOSTNAME", None)
    self.env.pop("ROS_NAMESPACE", None)
    # All future launch/reset/mapping/record/run/probe calls must use these kwargs.
    self.subprocess_kwargs = {"cwd": str(self.workspace), "env": self.env,
                              "start_new_session": True}
    self.process_logs = {}
    self.process_descendants = {}  # Cleanup must retain descendants that change sessions.
    metadata = {"world": self.world, "baseline": self.baseline, "repetition": self.repetition,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "workspace": str(self.workspace),
                "config_path": str(config_path), "commands": self.command_plan(),
                "environment": {key: self.env.get(key) for key in
                  ("ROS_DISTRO", "ROS_MASTER_URI", "GAZEBO_MASTER_URI", "ROS_IP", "ROS_HOME", "ROS_LOG_DIR")},
                "config": {key: value for key, value in self.config.items() if not key.startswith("_")}}
    with (self.attempt_dir / "metadata.yaml").open("x") as stream:
      yaml.safe_dump(metadata, stream, sort_keys=False)
    return self.attempt_dir

  def _probe(self, command, cwd):
    """Read setup command output with a timeout and no shell expansion."""
    try:
      result = subprocess.run(command, cwd=str(cwd), env=self.env, capture_output=True,
                              text=True, timeout=15, check=True)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
      raise ValueError(f"Setup command failed: {command}: {exc.stderr}") from exc
    return result.stdout.strip()

  def _start(self, name, command):
    import psutil
    log = (self.attempt_dir / f'{name}.log').open('xb')
    self.process_logs[name] = log
    kwargs = dict(self.subprocess_kwargs)
    kwargs['env'] = dict(self.env, TBAI_BENCHMARK_PROCESS=name)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, **kwargs)
    self.processes[name] = process
    self.process_descendants[name] = set()
    try:
      self.process_descendants[name].add(psutil.Process(process.pid))
    except psutil.NoSuchProcess:
      pass  # The next health check reports an immediate startup failure.
    return process

  def _track(self, scan=False):
    """Retain process identities, including children that start another session."""
    import psutil
    for owned in self.process_descendants.values():
      for process in list(owned):
        try:
          owned.update(process.children(recursive=True))
        except psutil.NoSuchProcess:
          pass
    if scan:
      # Find reparented children by the unique environment inherited at launch.
      for process in psutil.process_iter():
        try:
          env = process.environ()
          if env.get('TBAI_BENCHMARK_ATTEMPT') == str(self.attempt_dir):
            name = env.get('TBAI_BENCHMARK_PROCESS')
            if name in self.process_descendants:
              self.process_descendants[name].add(process)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
          pass

  def _health(self, allow_exit=()):
    self._track()
    for name, process in self.processes.items():
      if name not in allow_exit and process.poll() is not None:
        raise RuntimeError(f'{name} exited with code {process.returncode}; see {name}.log')

  def _snapshot(self):
    try:
      return json.loads((self.attempt_dir / 'observer.json').read_text())
    except FileNotFoundError:
      return {}

  def _wait(self, condition, timeout, reason, allow_exit=()):
    deadline = time.monotonic() + timeout
    while True:
      self._health(allow_exit)
      snapshot = self._snapshot()
      if condition(snapshot):
        return snapshot
      if time.monotonic() >= deadline:
        raise RuntimeError(reason)
      time.sleep(self.config['readiness']['poll_wall_sec'])

  def _fresh(self, snapshot):
    now = time.monotonic()
    state = snapshot.get('state')
    return (state is not None and snapshot.get('clock') is not None and
            now - state['wall'] <= self.config['readiness']['state_stale_wall_sec'] and
            now - snapshot.get('clock_wall', 0) <= self.config['readiness']['clock_stall_wall_sec'] and
            now - snapshot.get('wall', 0) <= self.config['readiness']['state_stale_wall_sec'])

  def launch_and_reset(self):
    """Launch, wait for ROS, run reset, verify standing, then start mapping."""
    plan = self.command_plan()
    readiness = self.config['readiness']
    self.stage = 'launch'
    port = urlparse(self.env['ROS_MASTER_URI']).port
    self._start('master', ['roscore', '-p', str(port)])
    self._start('observer', [sys.executable, str(Path(__file__).with_name('trial_ros.py').resolve()),
                             str(self.attempt_dir)])
    self._wait(lambda s: bool(s.get('wall')), self.config['startup_timeout_wall_sec'], 'master_timeout')
    self._start('launch', plan['launch'])
    self.stage = 'readiness'
    services = {'/gazebo/pause_physics', '/gazebo/unpause_physics',
                '/gazebo/set_model_state', '/gazebo/set_model_configuration'}
    self._wait(lambda s: self._fresh(s) and s['clock'] > 0 and services <= set(s.get('services', [])),
               self.config['startup_timeout_wall_sec'], 'startup_timeout')
    self.stage = 'reset_script'
    reset = self._start('reset', plan['reset'])
    self._wait(lambda s: reset.poll() is not None, readiness['reset_timeout_wall_sec'],
               'reset_timeout', allow_exit=('reset',))
    if reset.returncode:
      raise RuntimeError(f'reset_failed: exit {reset.returncode}; see reset.log')
    self.stage = 'verify_reset'
    reset_done = time.monotonic()
    standing_since = None
    previous_sim = None
    start_x, start_y = self.config['world_settings'][self.world]['start_position'][:2]

    def standing(snapshot):
      nonlocal standing_since, previous_sim
      if not self._fresh(snapshot) or snapshot['state']['wall'] <= reset_done:
        standing_since = None
        return False
      sim = snapshot['clock']
      values = snapshot['state']['values']
      stable = (len(values) == 36 and all(math.isfinite(v) for v in values) and
                math.hypot(values[3] - start_x, values[4] - start_y) <= readiness['reset_xy_tolerance_m'] and
                abs(values[0]) <= readiness['stable_stand_max_abs_roll_rad'] and
                abs(values[1]) <= readiness['stable_stand_max_abs_pitch_rad'] and
                math.sqrt(sum(v * v for v in values[9:12])) <= readiness['stable_stand_max_base_speed_mps'] and
                values[5] > self.config['world_settings'][self.world]['failure']['fall']['min_base_height_world_m'])
      if not stable or (previous_sim is not None and sim < previous_sim):
        standing_since = None
      elif standing_since is None:
        standing_since = sim
      previous_sim = sim
      return standing_since is not None and sim - standing_since >= readiness['stable_stand_hold_sim_sec']

    self._wait(standing, readiness['reset_timeout_wall_sec'], 'reset_verification_timeout', ('reset',))
    self.stage = 'mapping'
    self._start('mapping', plan['mapping'])


  def record_and_run(self):
    """Start the recorder, arm the monitor, and start motion."""
    from trial_monitor import TrialMonitor
    self.stage = 'record'
    plan = self.command_plan()
    self._start('record', plan['record'] + ['__name:=benchmark_recorder'])
    self.stage = 'arm_monitor'
    self.monitor = TrialMonitor(self.config, self.world)
    (self.attempt_dir / 'arm').touch()
    self._wait(lambda s: s.get('armed', False), self.config['activation_timeout_wall_sec'],
               'monitor_arm_timeout', ('reset',))
    self.stage = 'run_script'
    self._start('run', plan['run'])
    self.stage = 'monitor'
    return self.monitor.wait_for_result(self._snapshot, lambda: self._health(('reset',)))

  def _stop(self, name):
    import psutil
    if name not in self.processes:
      return
    settings = self.config['cleanup']
    steps = [(signal.SIGINT, settings['recorder_sigint_timeout_wall_sec'] if name == 'record'
              else settings['process_sigint_timeout_wall_sec']),
             (signal.SIGTERM, settings['process_sigterm_timeout_wall_sec']),
             (signal.SIGKILL, settings['process_sigkill_timeout_wall_sec'])]

    def alive():
      self.processes[name].poll()  # Reap the direct child.
      running = []
      for process in self.process_descendants[name]:
        try:
          if process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
            running.append(process)
        except psutil.NoSuchProcess:
          pass
      return running

    for sig, timeout in steps:
      self._track(scan=True)
      targets = alive()
      if not targets:
        return
      for process in targets:
        try:
          process.send_signal(sig)
        except psutil.NoSuchProcess:
          pass
      deadline = time.monotonic() + timeout
      while time.monotonic() < deadline:
        if not alive():
          return
        time.sleep(0.05)
    if alive():
      raise RuntimeError(f'{name}: processes still alive after SIGKILL')

  def cleanup(self):
    """Stop motion, finalize the bag, then stop all other owned processes."""
    errors = []

    def stop(name):
      try:
        self._stop(name)
      except Exception as exc:
        errors.append(f'{name}: {exc}')

    stop('run')
    stop('reset')
    if 'observer' in self.processes and 'launch' in self.processes:
      try:
        (self.attempt_dir / 'zero').touch()
        deadline = time.monotonic() + self.config['cleanup']['process_sigint_timeout_wall_sec']
        while not self._snapshot().get('zero_sent', False):
          if self.processes['observer'].poll() is not None or time.monotonic() >= deadline:
            raise RuntimeError('zero velocity could not be confirmed')
          time.sleep(0.05)
      except Exception as exc:
        errors.append(str(exc))
    stop('record')
    if 'record' in self.processes and not (self.attempt_dir / 'recording.bag').is_file():
      errors.append('recording.bag was not finalized; preserve recording.bag.active')
    for name in ('mapping', 'launch', 'observer', 'master'):
      stop(name)
    # One final sweep catches descendants reparented during shutdown.
    for name in self.processes:
      stop(name)
    for log in self.process_logs.values():
      try:
        log.close()
      except OSError as exc:
        errors.append(f'closing process log: {exc}')
    return errors

  def execute(self):
    """Always clean up and save the outcome, including partial startup failures."""
    from trial_monitor import TrialResult
    from dataclasses import asdict
    from trial_metrics import TrialMetrics, write_json
    previous_handlers = {}

    def interrupted(signum, frame):
      raise KeyboardInterrupt(f'signal {signum}')

    result = None
    cleanup_errors = []
    try:
      for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, interrupted)
      self.prepare()
      self.launch_and_reset()
      result = self.record_and_run()
    except (Exception, KeyboardInterrupt) as exc:
      status = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'error'
      if self.monitor is not None:
        result = self.monitor.finish(status, str(exc), self._snapshot().get('clock'),
                                     time.monotonic(), self.stage)
      else:
        result = TrialResult(status, str(exc), self.stage)
    finally:
      # A second Ctrl+C must not abandon bag finalization or process cleanup.
      for sig in previous_handlers:
        signal.signal(sig, signal.SIG_IGN)
      try:
        try:
          cleanup_errors = self.cleanup()
        except Exception as exc:
          cleanup_errors = [f'cleanup failed: {exc}']
      finally:
        for sig, handler in previous_handlers.items():
          signal.signal(sig, handler)
    result.cleanup_errors.extend(cleanup_errors)
    if hasattr(self, 'attempt_dir'):
      # Preserve the outcome before potentially expensive bag analysis.
      write_json(self.attempt_dir / 'result.json', asdict(result))
      metrics = TrialMetrics()
      summary = {'analysis_error': 'bag unavailable'}
      bag = self.attempt_dir / 'recording.bag'
      if bag.is_file():
        try:
          summary = metrics.summarize(bag, result, self.config, self.world)
        except Exception as exc:
          summary = {'analysis_error': str(exc)}
      metrics.save(self.attempt_dir, result, summary)
    return result
