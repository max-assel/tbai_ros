#!/usr/bin/env python3
"""Preview a benchmark matrix; execution remains blocked by explicit TODO hooks."""

import argparse
from pathlib import Path

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
  # TODO: validate positive timeouts, poses, goals, failure rules and recording topics.
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
