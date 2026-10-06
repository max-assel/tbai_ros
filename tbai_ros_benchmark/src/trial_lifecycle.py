from pathlib import Path
import json
import os
import signal
from dataclasses import asdict
import tempfile
import math
import subprocess
import time
from threading import Event

import rosgraph
import rospy
import rosservice
from geometry_msgs.msg import Twist
from tbai_ros_msgs.msg import RbdState

from trial_monitor import TrialMonitor, TrialResult

CONTROLLERS = {"MPC": "tbai_ros_mpc", "RL": "tbai_ros_bob", "DTC": "tbai_ros_dtc"}
SERVICES = {"/gazebo/pause_physics", "/gazebo/unpause_physics",
            "/gazebo/set_model_state", "/gazebo/set_model_configuration"}


class TrialLifecycle:

    def __init__(self, config, world, controller, repetition):
        self.config, self.world, self.controller = config, world, controller
        self.repetition = repetition
        self.processes = {}
        self.process_logs = {}
        self.stage = "prepare"
        self.motion = False
        self.settings = config["robot_readiness"]
        self.latest = None
        self.last_state_advance = None
        self.subscribers = []

    def prepare(self):
        if not hasattr(self, "attempt_dir"):
            output_dir = Path(self.config["output_dir"])
            if not output_dir.is_absolute():
                config_dir = Path(self.config["_config_path"]).parent
                output_dir = config_dir / output_dir
            trial_root = output_dir / self.world / self.controller
            trial_root.mkdir(parents=True, exist_ok=True)
            self.attempt_dir = Path(tempfile.mkdtemp(
                prefix=f"trial_{self.repetition:03d}_", dir=trial_root
            ))
    def command_plan(self):
        cleanup = self.config["cleanup"]
        roslaunch = ["roslaunch", "--sigint-timeout", str(cleanup["process_sigint_timeout_wall_sec"]),
                     "--sigterm-timeout", str(cleanup["process_sigterm_timeout_wall_sec"])]
        return {
        "launch": [*roslaunch, CONTROLLERS[self.controller], "anymal_d_perceptive.launch",
                    f"world:={self.world}", f"gui:={str(self.config['gazebo_gui']).lower()}",
                    f"rviz:={str(self.config.get('rviz', True)).lower()}"],
        "reset": ["bash", "-e", self.config["scripts"]["reset"], self.world, self.controller],
        "mapping": [*roslaunch, "tbai_ros_gridmap", "elevation_mapping.launch"],
        "record": ["rosbag", "record", "-O",
                    str(self.attempt_dir / "recording.bag"),
                    *self.config["record_topics"]],
        "run": ["bash", "-e", self.config["scripts"]["run"], self.world, self.controller],
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
        return (set(self.config["record_topics"]) <= topics
                and (self.attempt_dir / "recording.bag.active").exists())

    def monitor_until_finished(self):
        armed = time.monotonic()
        while True:
            now = time.monotonic()
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
        for sig, limit in ((signal.SIGINT, first),
                           (signal.SIGTERM, "process_sigterm_timeout_wall_sec"),
                           (signal.SIGKILL, "process_sigkill_timeout_wall_sec")):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                process.wait()
                return
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
                    except (ProcessLookupError, FileNotFoundError):
                        continue
                    fields = stat[stat.rfind(")") + 2:].split()
                    # /proc/<pid>/stat fields: state, parent PID, process group.
                    # Zombies have exited and cannot respond to more signals.
                    if int(fields[2]) == process.pid and fields[0] not in ("Z", "X"):
                        break
                else:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.05, remaining))
        raise RuntimeError(f"{name} process group did not stop")

    def cleanup(self):
        self.stage = "cleanup"
        errors = []
        for name in ("run", "reset", "record", "mapping", "launch"):
            try:
                self.stop(name)
            except (OSError, RuntimeError) as exc:
                errors.append(str(exc))
        for subscriber in self.subscribers:
            subscriber.unregister()
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
            self.stage = "reset_script"
            self.start("reset", plan["reset"])
            self.processes["reset"].wait(timeout=self.settings["reset_timeout_wall_sec"])
            if self.processes["reset"].returncode != 0:
                raise RuntimeError("reset script failed")
            self.processes.pop("reset")
            self.standing_since = None
            self.latest = None
            self.last_state_advance = None
            self.stage = "verify_reset"
            self.wait(self.standing, self.settings["reset_timeout_wall_sec"], "stable standing")

            self.stage = "mapping"
            map_received = Event()
            map_subscriber = rospy.Subscriber(
                "/elevation_mapping/elevation_map_raw", rospy.AnyMsg,
                lambda msg: map_received.set(), queue_size=1,
            )
            try:
                self.start("mapping", plan["mapping"])
                self.wait(map_received.is_set, self.settings["mapping_timeout_wall_sec"], "mapping output")
            finally:
                map_subscriber.unregister()

            self.subscribers.append(rospy.Subscriber(
                self.config["monitor"]["motion_topic"], Twist, self.on_motion, queue_size=1,
            ))
            self.stage = "record"
            self.start("record", plan["record"] + ["__name:=benchmark_recorder"])
            self.wait(self.recording_ready, self.config["activation_timeout_wall_sec"], "recording ready")
            self.stage = "arm_monitor"
            self.monitor = TrialMonitor(self.config, self.world)
            result_factory = self.monitor.result
            self.motion = False
            self.stage = "run_script"
            self.start("run", plan["run"])
            self.stage = "monitor"
            result = self.monitor_until_finished()
        except KeyboardInterrupt:
            result = result_factory("interrupted", f"interrupted during {self.stage}")
        except Exception as exc:
            result = result_factory("error", f"{self.stage}: {exc}")
        finally:
            handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                result.cleanup_errors = self.cleanup()
            finally:
                signal.signal(signal.SIGINT, handler)
        self.stage = "summarize"
        (self.attempt_dir / "result.json").write_text(json.dumps(asdict(result), indent=2) + "\n")
        return result
