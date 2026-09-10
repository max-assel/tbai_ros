#!/usr/bin/env python3
"""Preview a benchmark matrix; execution remains blocked by explicit TODO hooks."""

import argparse
import math
from pathlib import Path
import re

import yaml

from trial_lifecycle import BASELINES, STAGES, TrialLifecycle
from trial_monitor import TrialResult


def load_config(path):
  """Validate the matrix needed by the preview; extend for execution settings."""
  with path.open() as stream:
    config = yaml.safe_load(stream)
  if not isinstance(config, dict):
    raise ValueError("Config must be a YAML mapping")
  for key in ("worlds", "baselines"):
    values = config.get(key)
    if not isinstance(values, list) or not values or not all(isinstance(v, str) for v in values):
      raise ValueError(f"{key} must be a nonempty list of names")
    if len(set(values)) != len(values):
      raise ValueError(f"{key} must not contain duplicates")
  count = config.get("repetitions")
  if type(count) is not int or count < 1:
    raise ValueError("repetitions must be a positive integer")
  if any(b not in BASELINES for b in config["baselines"]):
    raise ValueError("Supported baselines: MPC, RL, DTC")
  settings = config.get("world_settings", {})
  if not isinstance(settings, dict) or any(w not in settings for w in config["worlds"]):
    raise ValueError("Each world needs a world_settings entry")
  reset_mode = config.get("reset_mode")
  if reset_mode not in ("restart", "reuse"):
    raise ValueError("reset_mode must be restart or reuse")
  reset_equivalence_verified = config.get("reset_equivalence_verified", False)
  if type(reset_equivalence_verified) is not bool:
    raise ValueError("reset_equivalence_verified must be a boolean")
  if reset_mode == "reuse" and not reset_equivalence_verified:
    raise ValueError("reuse requires reset_equivalence_verified: true")
  def mapping(value, name):
    if not isinstance(value, dict):
      raise ValueError(f"{name} must be a mapping")
    return value

  def number(value, name, minimum=None, strict=False):
    if type(value) not in (int, float) or not math.isfinite(value):
      raise ValueError(f"{name} must be a finite number")
    if minimum is not None and (value < minimum or (strict and value == minimum)):
      relation = "greater than" if strict else "at least"
      raise ValueError(f"{name} must be {relation} {minimum}")

  def vector(value, name, size):
    if not isinstance(value, list) or len(value) != size:
      raise ValueError(f"{name} must contain {size} numbers")
    for index, component in enumerate(value):
      number(component, f"{name}[{index}]")
    return value

  def bounds(value, name):
    for axis in ("x_bounds_m", "y_bounds_m"):
      lower, upper = vector(value.get(axis), f"{name}.{axis}", 2)
      if lower >= upper:
        raise ValueError(f"{name}.{axis} must have increasing bounds")

  for key in ("startup_timeout_wall_sec", "trial_timeout_sim_sec", "trial_timeout_wall_sec"):
    number(config.get(key), key, 0, strict=True)
  success = mapping(config.get("success"), "success")
  for key in ("xy_tolerance_m", "hold_sim_sec"):
    number(success.get(key), f"success.{key}", 0, strict=True)

  for world in config["worlds"]:
    name = f"world_settings.{world}"
    entry = mapping(settings[world], name)
    for key in ("start_position", "goal_position"):
      vector(entry.get(key), f"{name}.{key}", 3)
    orientation = vector(entry.get("start_orientation_xyzw"), f"{name}.start_orientation_xyzw", 4)
    if not math.isclose(math.hypot(*orientation), 1.0, rel_tol=1e-3):
      raise ValueError(f"{name}.start_orientation_xyzw must be a unit quaternion")
    failure = mapping(entry.get("failure"), f"{name}.failure")
    for rule in ("fall", "course_boundary", "no_progress"):
      rule_name = f"{name}.failure.{rule}"
      values = mapping(failure.get(rule), rule_name)
      if rule == "fall":
        number(values.get("min_base_height_world_m"), f"{rule_name}.min_base_height_world_m")
        for key in ("max_abs_roll_rad", "max_abs_pitch_rad"):
          number(values.get(key), f"{rule_name}.{key}", 0, strict=True)
          if values[key] > math.pi:
            raise ValueError(f"{rule_name}.{key} must not exceed pi")
      elif rule == "course_boundary":
        if values.get("frame") != "world":
          raise ValueError(f"{rule_name}.frame must be world")
        bounds(values, rule_name)
        segments = values.get("segments", [])
        if not isinstance(segments, list):
          raise ValueError(f"{rule_name}.segments must be a list")
        for index, segment in enumerate(segments):
          segment_name = f"{rule_name}.segments[{index}]"
          bounds(mapping(segment, segment_name), segment_name)
      else:
        if values.get("metric") != "best_goal_xy_distance_reduction":
          raise ValueError(f"{rule_name}.metric must be best_goal_xy_distance_reduction")
        for key in ("min_progress_m", "window_sim_sec"):
          number(values.get(key), f"{rule_name}.{key}", 0, strict=True)
        number(values.get("grace_sim_sec"), f"{rule_name}.grace_sim_sec", 0)
        if type(values.get("exclude_goal_tolerance")) is not bool:
          raise ValueError(f"{rule_name}.exclude_goal_tolerance must be a boolean")
      if rule != "no_progress":
        number(values.get("hold_sim_sec"), f"{rule_name}.hold_sim_sec", 0, strict=True)

  topics = config.get("record_topics")
  if not isinstance(topics, list) or not topics:
    raise ValueError("record_topics must be a nonempty list of ROS topic names")
  for topic in topics:
    if not isinstance(topic, str) or not re.fullmatch(r"[A-Za-z/~][A-Za-z0-9_/]*", topic) or topic in ("/", "~"):
      raise ValueError("record_topics must contain valid ROS topic names")
  if len(set(topics)) != len(topics):
    raise ValueError("record_topics must not contain duplicates")

  return config


def run_trial(lifecycle):
  """Illustrate orchestration and unconditional cleanup for a single attempt."""
  result = TrialResult("error", "Trial did not finish")
  try:
    lifecycle.start_stack()
    lifecycle.wait_for_robot()
    lifecycle.reset_robot()
    lifecycle.start_mapping()
    lifecycle.wait_for_map()
    lifecycle.start_recording()
    lifecycle.start_motion()
    result = lifecycle.monitor.wait_for_result()
  except KeyboardInterrupt:
    result = TrialResult("interrupted", "User interrupted the batch")
    raise
  except Exception as exc:
    result = TrialResult("error", str(exc))
    raise  # Stop batch on infrastructure errors until retry policy is implemented.
  finally:
    try:
      lifecycle.cleanup()
    except Exception as exc:
      result = TrialResult("error", f"Prior outcome: {result.status}: {result.reason}; cleanup failed: {exc}")
      raise  # Never start another trial after uncertain cleanup.
    finally:
      lifecycle.save_result(result)


def main():
  """Print all attempts by default, without importing ROS or writing results."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/benchmark.yaml")
  mode = parser.add_mutually_exclusive_group()
  mode.add_argument("--dry-run", action="store_true", help="Preview only (the default)")
  mode.add_argument("--execute", action="store_true", help="Requires implemented lifecycle and monitor hooks")
  args = parser.parse_args()
  try:
    config = load_config(args.config)
    if args.execute:
      TrialLifecycle.check_implementation()
    for world in config["worlds"]:
      for baseline in config["baselines"]:
        for repetition in range(1, config["repetitions"] + 1):
          print(f"{world}/{baseline}/trial_{repetition:03d}: {' -> '.join(STAGES)}")
          if args.execute:
            run_trial(TrialLifecycle(config, world, baseline, repetition))
  except (OSError, ValueError, yaml.YAMLError, NotImplementedError) as exc:
    parser.exit(2, f"benchmark_runner: {exc}\n")


if __name__ == "__main__":
  main()
