from pathlib import Path
import json
import os
import signal
import socket
from dataclasses import asdict
import tempfile
import math
import subprocess
import time
from threading import Event
from urllib.parse import urlparse

import rosgraph
import rospy
import rosservice
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tbai_ros_msgs.msg import JointCommandArray, RbdState

from trial_monitor import TrialMonitor, TrialResult

CONTROLLERS = {"MPC": "tbai_ros_mpc", "RL": "tbai_ros_bob", "DTC": "tbai_ros_dtc"}
SERVICES = {"/gazebo/pause_physics", "/gazebo/unpause_physics",
            "/gazebo/set_model_state", "/gazebo/set_model_configuration"}


class TrialLifecycle:

    def __init__(self, config, world, controller, repetition, attempt=1):
        self.config, self.world, self.controller = config, world, controller
        self.repetition = repetition
        self.attempt = attempt
        self.processes = {}
        self.process_logs = {}
        self.stage = "prepare"
        self.motion = False
        self.motion_origin = None
        self.walking_started = False
        self.settings = config["robot_readiness"]
        self.latest = None
        self.last_state_advance = None
        self.subscribers = []
        self.sitting_since = None
        self.controller_publisher = None
        self.gait_publisher = None
        self.map_updates = 0
        self.map_started = None

    def prepare(self):
        if not hasattr(self, "attempt_dir"):
            output_dir = Path(self.config["output_dir"])
            if not output_dir.is_absolute():
                config_dir = Path(self.config["_config_path"]).parent
                output_dir = config_dir / output_dir
            trial_root = output_dir / self.world / self.controller
            trial_root.mkdir(parents=True, exist_ok=True)
            self.attempt_dir = Path(tempfile.mkdtemp(
                prefix=f"trial_{self.repetition:03d}_attempt_{self.attempt}_", dir=trial_root
            ))
    def command_plan(self):
        robot = self.config.get("robot", "anymal_d")
        if robot not in ("anymal_d", "go2"):
            raise ValueError(f"Unsupported benchmark robot: {robot}")
        cleanup = self.config["cleanup"]
        roslaunch = ["roslaunch", "--sigint-timeout", str(cleanup["process_sigint_timeout_wall_sec"]),
                     "--sigterm-timeout", str(cleanup["process_sigterm_timeout_wall_sec"])]
        mapping_launch = ["tbai_ros_gridmap", "elevation_mapping.launch"]
        if robot == "go2" and self.controller == "MPC":
            mapping_launch = ["tbai_ros_mpc", "go2_mapping.launch"]
        start = self.config["world_settings"][self.world]["start_position"]
        return {
        "launch": [*roslaunch, CONTROLLERS[self.controller], f"{robot}_perceptive.launch",
                    f"world:={self.world}", f"gui:={str(self.config['gazebo_gui']).lower()}",
                    f"rviz:={str(self.config.get('rviz', True)).lower()}",
                    f"x:={start[0]}", f"y:={start[1]}"],
        "reset": ["bash", "-e", self.config["scripts"]["reset"], self.world, self.controller, "--no-stand"],
        "mapping": [*roslaunch, *mapping_launch],
        "record": ["rosbag", "record", "-O",
                    str(self.attempt_dir / "recording.bag"),
                    *self.config.get("record_topics", [])],
        "run": ["bash", "-e", self.config["scripts"]["run"], self.world, self.controller, "--no-controller"],
        }
    
    def on_state(self, msg):
        stamp = msg.stamp.to_sec()
        values = tuple(msg.rbd_state)
        if not all(math.isfinite(value) for value in values):
            return
        now = time.monotonic()
        previous = self.latest
        if previous is not None and stamp > previous[0]:
            self.last_state_advance = now
        origin = self.motion_origin
        if origin is not None:
            # Latch actual movement, including sideways/backwards movement or
            # stepping in place. A command alone does not prove the robot walked.
            self.walking_started |= (
                math.hypot(values[3] - origin[3], values[4] - origin[4]) > 0.02
                or any(abs(a - b) > 0.1 for a, b in zip(values[12:24], origin[12:24]))
            )
        self.latest = (stamp, values, now)

    def fresh(self):
        state = self.latest
        now = time.monotonic()
        limit = self.settings["state_stale_wall_sec"]
        return (state is not None
                and now - state[2] < limit
                and self.last_state_advance is not None
                and now - self.last_state_advance < limit)

    def wait(self, cond, timeout, reason):
        deadline = time.monotonic() + timeout
        while True:
            if rospy.is_shutdown():
                raise RuntimeError(f"ROS shut down during {self.stage}")
            for name, process in self.processes.items():
                if process.poll() is not None:
                    raise RuntimeError(
                        f"{name} exited ({process.returncode}) during {self.stage}"
                    )
            if time.monotonic() >= deadline:
                raise RuntimeError(f"timeout: {reason}")
            if cond():
                return
            time.sleep(self.settings["poll_wall_sec"])

    def sitting(self):
        state = self.latest
        joint_tolerance = self.settings.get("sit_joint_tolerance_rad", 0.1)
        if (state is None or not self.fresh()
                or not all(abs(actual - target) < joint_tolerance
                           for actual, target in zip(state[1][12:24], self.sit_joint_positions))
                or not all(abs(speed) < 0.2 for speed in state[1][24:36])):
            self.sitting_since = None
            return False
        stamp = state[0]
        if self.sitting_since is None or stamp < self.sitting_since:
            self.sitting_since = stamp
        return stamp - self.sitting_since >= 0.2

    def standing(self):
        state = self.latest
        if state is None or not self.fresh():
            self.standing_since = None
            return False

        stamp, values, _ = state
        roll, pitch = values[:2]
        x, y = values[3:5]
        speed = math.sqrt(sum(v * v for v in values[9:12]))
        start = self.config["world_settings"][self.world]["start_position"]
        stable = (
            math.hypot(x - start[0], y - start[1])
            <= self.settings["reset_xy_tolerance_m"]
            and abs(roll) <= self.settings["stable_stand_max_abs_roll_rad"]
            and abs(pitch) <= self.settings["stable_stand_max_abs_pitch_rad"]
            and speed <= self.settings["stable_stand_max_base_speed_mps"]
            # A sitting robot can also be level and stationary.
            and all(abs(actual - target) <= 0.2
                    for actual, target in zip(values[12:24], self.stand_joint_positions))
        )
        if not stable:
            self.standing_since = None
            return False
        if self.standing_since is None or stamp < self.standing_since:
            self.standing_since = stamp
        return (stamp - self.standing_since
                >= self.settings["stable_stand_hold_sim_sec"])

    def start(self, name, command):
        log = (self.attempt_dir / f"{name}.log").open("w")
        self.process_logs[name] = log
        self.processes[name] = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, "TBAI_TRIAL_PROCESS": str(self.attempt_dir / name)},
        )

    def on_motion(self, msg):
        linear = math.sqrt(sum(v * v for v in (msg.linear.x, msg.linear.y, msg.linear.z)))
        angular = math.sqrt(sum(v * v for v in (msg.angular.x, msg.angular.y, msg.angular.z)))
        settings = self.config["monitor"]
        self.motion |= (linear > settings["motion_linear_epsilon_mps"] or
                        angular > settings["motion_angular_epsilon_radps"])

    def recording_ready(self):
        _, subscriptions, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        topics = {topic for topic, nodes in subscriptions if "/benchmark_recorder" in nodes}
        # Mapping and walking publishers start later. Only require the configured
        # state topic now so recording can capture reset and startup failures.
        state_topic = "/" + self.config["monitor"]["state_topic"].lstrip("/")
        required = {state_topic} & set(self.config["record_topics"])
        return (required <= topics
                and (self.attempt_dir / "recording.bag.active").exists())

    def activate_controller(self):
        self.stage = "activate_controller"
        timeout = self.config["activation_timeout_wall_sec"]
        self.wait(lambda: self.controller_publisher.get_num_connections() > 0,
                  timeout, "controller command subscriber")
        mode = {"MPC": "WBC", "RL": "BOB", "DTC": "DTC"}[self.controller]
        self.controller_publisher.publish(String(data=mode))
        if self.controller != "RL":
            ready_at = time.monotonic() + 1
            self.wait(lambda: time.monotonic() >= ready_at, timeout, "controller startup delay")
            self.gait_publisher = rospy.Publisher("/gait_command", String, queue_size=1, latch=True)
            self.wait(lambda: self.gait_publisher.get_num_connections() > 0,
                      timeout, "gait command subscriber")
            gait = "go2_trot" if self.controller == "MPC" and self.config.get("robot") == "go2" else "trot"
            self.gait_publisher.publish(String(data=gait))
        ready_at = time.monotonic() + 1
        self.wait(lambda: time.monotonic() >= ready_at, timeout, "walking startup delay")

    def monitor_until_finished(self):
        armed = time.monotonic()
        while True:
            now = time.monotonic()
            if self.motion and self.map_started is None:
                self.map_started = now
            if rospy.is_shutdown():
                return self.monitor.result("interrupted", "ROS shutdown")
            if not self.fresh():
                return self.monitor.result("error", "state stopped advancing")
            result = self.monitor.update(self.latest) if self.motion else None
            if result is not None:
                return result
            for name, process in self.processes.items():
                if process.poll() is not None:
                    return self.monitor.result("error", f"{name} exited ({process.returncode})")
            if not self.motion and now - armed >= self.config["activation_timeout_wall_sec"]:
                return self.monitor.result("error", "motion activation timeout")
            if now - armed >= self.config["timeout_wall_sec"]:
                return self.monitor.result("timeout", "wall_duration_limit")
            time.sleep(self.settings["poll_wall_sec"])

    def stop(self, name):
        process = self.processes.get(name)
        if process is None:
            return
        settings = self.config["cleanup"]
        first = "recorder_sigint_timeout_wall_sec" if name == "record" else "process_sigint_timeout_wall_sec"
        groups = {process.pid}
        # Descendants inherit this marker even if wrappers exit or Gazebo
        # creates another process group. Never match unrelated simulations.
        marker = os.fsencode(f"TBAI_TRIAL_PROCESS={self.attempt_dir / name}")
        for stat_path in Path("/proc").glob("[0-9]*/stat"):
            try:
                if marker in (stat_path.parent / "environ").read_bytes().split(b"\0"):
                    stat = stat_path.read_text()
                    fields = stat[stat.rfind(")") + 2:].split()
                    groups.add(int(fields[2]))
            except (ProcessLookupError, FileNotFoundError, PermissionError):
                continue
        for sig, limit in ((signal.SIGINT, first),
                           (signal.SIGTERM, "process_sigterm_timeout_wall_sec"),
                           (signal.SIGKILL, "process_sigkill_timeout_wall_sec")):
            # Preserve roslaunch's graceful shutdown; escalate surviving nodes.
            for group in ({process.pid} if sig == signal.SIGINT else groups):
                try:
                    os.killpg(group, sig)
                except ProcessLookupError:
                    pass
            timeout = settings[limit]
            if sig == signal.SIGINT and name in ("launch", "mapping"):
                timeout += (settings["process_sigterm_timeout_wall_sec"]
                            + settings["process_sigkill_timeout_wall_sec"])
            deadline = time.monotonic() + timeout
            while True:
                process.poll()  # Reap the parent, but also wait for its children.
                for stat_path in Path("/proc").glob("[0-9]*/stat"):
                    try:
                        stat = stat_path.read_text()
                    except (ProcessLookupError, FileNotFoundError, PermissionError):
                        continue
                    fields = stat[stat.rfind(")") + 2:].split()
                    # /proc/<pid>/stat fields: state, parent PID, process group.
                    # Zombies have exited and cannot respond to more signals.
                    if int(fields[2]) in groups and fields[0] not in ("Z", "X"):
                        break
                else:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.05, remaining))
        raise RuntimeError(f"{name} process group did not stop")

    def gazebo_running(self):
        gazebo = urlparse(os.environ.get("GAZEBO_MASTER_URI", "http://localhost:11345"))
        try:
            with socket.create_connection((gazebo.hostname, gazebo.port or 11345), timeout=0.5):
                return True
        except OSError:
            return False

    def cleanup(self):
        self.stage = "cleanup"
        errors = []
        for name in ("run", "reset", "record", "mapping", "launch"):
            try:
                self.stop(name)
            except (OSError, RuntimeError) as exc:
                errors.append(str(exc))
        if "launch" in self.processes:
            deadline = time.monotonic() + self.config["cleanup"]["process_sigkill_timeout_wall_sec"]
            while self.gazebo_running():
                if time.monotonic() >= deadline:
                    errors.append("Gazebo port is still in use after trial cleanup")
                    break
                time.sleep(self.settings["poll_wall_sec"])
        for subscriber in self.subscribers:
            subscriber.unregister()
        for publisher in (self.controller_publisher, self.gait_publisher):
            if publisher is not None:
                publisher.unregister()
        self.controller_publisher = self.gait_publisher = None
        for log in self.process_logs.values():
            log.close()
        return errors

    def execute(self):
        self.prepare()
        plan = self.command_plan()

        self.subscribers = []
        result_factory = TrialResult
        result = TrialResult("error", "trial did not complete")
        try:
            # Local trials must not advertise an unresolvable laptop hostname.
            if urlparse(os.environ.get("ROS_MASTER_URI", "http://localhost:11311")).hostname in ("localhost", "127.0.0.1"):
                os.environ.pop("ROS_HOSTNAME", None)
                os.environ["ROS_IP"] = "127.0.0.1"
            if self.gazebo_running():
                raise RuntimeError("Gazebo port is already in use; stop the existing simulation before starting trials")
            self.start("launch", plan["launch"])
            if not rospy.core.is_initialized():
                rospy.init_node("trial_executor", anonymous=True, disable_signals=True)
            self.subscribers.append(rospy.Subscriber(
                self.config["monitor"]["state_topic"], RbdState,
                self.on_state, queue_size=1,
            ))
            self.stage = "readiness"
            self.wait(
                lambda: self.fresh() and SERVICES <= set(rosservice.get_service_list()),
                self.config["startup_timeout_sec"], "services and state",
            )
            if self.config.get("record_rosbag", True):
                self.stage = "record"
                self.start("record", plan["record"] + ["__name:=benchmark_recorder"])
                self.wait(self.recording_ready, self.config["activation_timeout_wall_sec"], "recording ready")
            # Correct the spawn pose before waiting for the initial SIT to settle.
            self.stage = "reset_script"
            self.start("reset", plan["reset"])
            self.processes["reset"].wait(timeout=self.settings["reset_timeout_wall_sec"])
            if self.processes["reset"].returncode != 0:
                raise RuntimeError("reset script failed")
            self.processes.pop("reset")
            self.latest = None
            self.last_state_advance = None
            self.sit_joint_positions = rospy.get_param("/static_controller/sit_controller/joint_angles")
            joint_names = rospy.get_param("/joint_names")
            if len(self.sit_joint_positions) != 12 or len(joint_names) != 12:
                raise RuntimeError("Expected 12 sitting joint positions and joint names")
            sit_targets = dict(zip(joint_names, self.sit_joint_positions))
            sit_command_ready = Event()

            def on_initial_command(msg):
                if (len(msg.joint_commands) == len(sit_targets)
                        and all(command.joint_name in sit_targets
                                and abs(command.desired_position - sit_targets[command.joint_name]) < 0.001
                                for command in msg.joint_commands)):
                    sit_command_ready.set()

            command_subscriber = rospy.Subscriber(
                rospy.get_param("/command_topic"), JointCommandArray,
                on_initial_command, queue_size=1,
            )
            try:
                # Keep controller selection alive through walking activation.
                self.controller_publisher = rospy.Publisher(
                    rospy.get_param("/change_controller_topic"), String, queue_size=1, latch=True,
                )
                self.wait(sit_command_ready.is_set, self.config["startup_timeout_sec"], "initial SIT command")
                self.stage = "settle_sit"
                self.wait(self.sitting, self.settings["reset_timeout_wall_sec"], "initial SIT joints to settle")
            finally:
                command_subscriber.unregister()
            self.standing_since = None
            self.stage = "verify_reset"
            self.stand_joint_positions = rospy.get_param("/static_controller/stand_controller/joint_angles")
            if len(self.stand_joint_positions) != 12:
                raise RuntimeError("Expected 12 standing joint positions")
            self.wait(self.fresh, self.settings["reset_timeout_wall_sec"], "fresh state after reset")
            self.wait(lambda: self.controller_publisher.get_num_connections() > 0,
                      self.config["activation_timeout_wall_sec"], "STAND command subscriber")
            self.controller_publisher.publish(String(data="STAND"))
            self.wait(self.standing, self.settings["reset_timeout_wall_sec"], "stable standing")

            self.stage = "mapping"
            map_received = Event()

            def on_map(msg):
                if self.map_started is not None:
                    self.map_updates += 1
                map_received.set()

            self.subscribers.append(rospy.Subscriber(
                "/elevation_mapping/elevation_map_raw", rospy.AnyMsg,
                on_map, queue_size=1,
            ))
            self.start("mapping", plan["mapping"])
            self.wait(map_received.is_set, self.settings["mapping_timeout_wall_sec"], "mapping output")

            self.subscribers.append(rospy.Subscriber(
                self.config["monitor"]["motion_topic"], Twist, self.on_motion, queue_size=1,
            ))
            self.stage = "arm_monitor"
            self.monitor = TrialMonitor(self.config, self.world)
            result_factory = self.monitor.result
            self.motion = False
            self.motion_origin = self.latest[1]
            self.activate_controller()
            self.stage = "run_script"
            self.start("run", plan["run"])
            self.stage = "monitor"
            result = self.monitor_until_finished()
        except KeyboardInterrupt:
            result = result_factory("interrupted", f"interrupted during {self.stage}")
        except Exception as exc:
            result = result_factory("error", f"{self.stage}: {exc}")
        finally:
            self.motion_origin = None  # Ignore movement caused by shutdown.
            if self.map_started is not None:
                elapsed = time.monotonic() - self.map_started
                self.map_started = None
                result.map_update_rate_wall_hz = self.map_updates / elapsed if elapsed > 0 else None
            handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                result.cleanup_errors = self.cleanup()
                result.retryable = (result.status in ("error", "failure", "timeout")
                                    and not self.walking_started and not result.cleanup_errors)
            finally:
                signal.signal(signal.SIGINT, handler)
        self.stage = "summarize"
        (self.attempt_dir / "result.json").write_text(json.dumps(asdict(result), indent=2) + "\n")
        return result
