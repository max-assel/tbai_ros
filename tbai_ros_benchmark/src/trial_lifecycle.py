"""Fill-in hooks for ROS/Gazebo orchestration. No processes are launched yet."""

from trial_monitor import TrialMonitor

BASELINES = {
  "MPC": {"package": "tbai_ros_mpc", "controller": "WBC", "gait": "trot"},
  "RL": {"package": "tbai_ros_bob", "controller": "BOB", "gait": None},
  "DTC": {"package": "tbai_ros_dtc", "controller": "DTC", "gait": "trot"},
}

STAGES = (
  "start_stack", "wait_for_robot", "reset_robot", "start_mapping",
  "wait_for_map", "start_recording", "start_motion", "monitor", "cleanup", "save_result",
)


class TrialLifecycle:
  """Own all resources for one attempt, including partially started resources."""

  def __init__(self, config, world, baseline, repetition):
    self.config = config
    self.world = world
    self.baseline = baseline
    self.repetition = repetition
    self.processes = []
    self.monitor = TrialMonitor(config, world)

  @staticmethod
  def check_implementation():
    """Fail before launching anything until the execution hooks are implemented."""
    # TODO: replace this guard with preflight checks AFTER implementing every hook.
    # Check ROS setup, launch/world paths, writable output, disk space, and configuration.
    raise NotImplementedError("Skeleton only: implement trial_lifecycle.py and trial_monitor.py before execution")

  def start_stack(self):
    """Start an isolated ROS/Gazebo stack and register every owned process."""
    # TODO: create unique batch/attempt directories; snapshot config and code revision.
    # TODO: allocate separate ROS_MASTER_URI and GAZEBO_MASTER_URI; propagate to all children.
    # TODO: launch BASELINES[self.baseline]['package']/anymal_d_perceptive.launch.
    # TODO: add conditional RViz support to launch files before passing rviz:=false.
    # TODO: use Popen argument lists, start_new_session=True, per-process log files.
    # Track descendants too: roslaunch children may have separate process groups.
    raise NotImplementedError("Start simulator and controller")

  def wait_for_robot(self):
    """Wait for advancing clock, fresh state, robot model and required services."""
    # TODO: apply startup_timeout_wall_sec with time.monotonic(), checking process health.
    raise NotImplementedError("Robot readiness checks")

  def reset_robot(self):
    """Reset pose/joints, request STAND, and verify stable standing."""
    # TODO: port tbai_ros_gazebo/reset_gazebo.sh operations using checked service responses.
    # Use world_settings; zero twist, check controller readiness and bound every wait.
    # Ensure physics is unpaused on reset exceptions. Restart mode is the initial scope.
    raise NotImplementedError("Checked robot reset")

  def start_mapping(self):
    """Start fresh mapping after reset."""
    # TODO: verify roslaunch tbai_ros_gridmap elevation_mapping.launch for each baseline.
    raise NotImplementedError("Mapping startup")

  def wait_for_map(self):
    """Check fresh usable map coverage around the robot within a wall deadline."""
    raise NotImplementedError("Map readiness checks")

  def start_recording(self):
    """Record configured topics and confirm recorder readiness before motion."""
    raise NotImplementedError("Rosbag startup")

  def start_motion(self):
    """Activate controller/gait, then start the global path velocity generator."""
    # TODO: port tbai_ros_gazebo/run_experiment.sh, checking subscribers/activation.
    # Pass _world and _planner; use one authoritative goal config for generator and monitor.
    raise NotImplementedError("Controller activation and path execution")

  def cleanup(self):
    """Idempotently release even partially initialized resources; bound every wait."""
    # TODO: stop generator, publish zero motion, gracefully finalize rosbag.
    # TODO: stop mapping/controllers/Gazebo/master, then escalate only owned processes.
    # TODO: verify descendants exit, close logs, and report cleanup errors.
    # Never use killall or rosnode kill -a. Return only after isolation is restored.
    raise NotImplementedError("Owned-process cleanup")

  def save_result(self, result):
    """Persist one attempt atomically without overwriting previous attempts."""
    # TODO: serialize dataclasses.asdict(result), world/baseline/repetition and metadata.
    # TODO: append summary CSV; preserve failed/interrupted attempts and cleanup errors.
    # TODO: implement resume by validating config identity and existing terminal results.
    raise NotImplementedError("Result persistence")
