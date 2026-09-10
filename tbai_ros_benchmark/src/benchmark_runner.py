#!/usr/bin/env python3
"""Preview existing commands; real execution is intentionally a fill-in skeleton."""

import argparse
from pathlib import Path
import shlex

import yaml

from trial_lifecycle import BASELINES, STAGES, TrialLifecycle


def load_config(path):
  """Validate preview matrix and resolve script paths relative to configuration."""
  with path.open() as stream:
    config = yaml.safe_load(stream)
  if not isinstance(config, dict):
    raise ValueError("Config must be a mapping")
  for key in ("worlds", "baselines"):
    values = config.get(key)
    if not isinstance(values, list) or not values or not all(isinstance(v, str) for v in values):
      raise ValueError(f"{key} must be a nonempty list of names")
    if len(set(values)) != len(values):
      raise ValueError(f"{key} contains duplicates")
  if any(b not in BASELINES for b in config["baselines"]):
    raise ValueError("Supported baselines: MPC, RL, DTC")
  if type(config.get("repetitions")) is not int or config["repetitions"] < 1:
    raise ValueError("repetitions must be a positive integer")
  if not isinstance(config.get("world_settings"), dict) or any(
    w not in config["world_settings"] for w in config["worlds"]
  ):
    raise ValueError("Each world needs monitor settings")
  if not isinstance(config.get("scripts"), dict):
    raise ValueError("scripts must define reset and run paths")
  for key in ("reset", "run"):
    value = config["scripts"].get(key)
    if not isinstance(value, str) or not value:
      raise ValueError(f"scripts.{key} must be a path")
    config["scripts"][key] = str((path.resolve().parent / value).resolve())
  topics = config.get("record_topics")
  if not isinstance(topics, list) or not topics or not all(isinstance(t, str) and t.startswith("/") for t in topics):
    raise ValueError("record_topics must contain absolute topic names")
  if type(config.get("gazebo_gui")) is not bool:
    raise ValueError("gazebo_gui must be a boolean")
  # TODO: Validate finite positive deadlines, monitor thresholds, topic fields and output path.
  # Suggestion: validate before creating processes; check goals match the existing path generator.
  return config


def execute_batch(config):
  """Fill in sequential orchestration around existing scripts."""
  # TODO: Create one batch directory, loop matrix and invoke TrialLifecycle.execute().
  # Suggestion: persist stage-specific intermediate setup errors and partial artifacts.
  # Continue after robot failure/timeout only after verified cleanup; stop on uncertain cleanup.
  # TODO: Connect TrialMonitor and TrialMetrics to each attempt.
  # Suggestion: arm monitor/recorder before motion, finalize bag before analysis, keep every attempt.
  # Leave automatic retry, resume and parallel trials for later.
  raise NotImplementedError("Skeleton only: implement wrappers, monitor and metrics before --execute")


def main():
  """Preview without importing ROS, launching processes or writing results."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/benchmark.yaml")
  mode = parser.add_mutually_exclusive_group()
  mode.add_argument("--dry-run", action="store_true", help="Preview only (default)")
  mode.add_argument("--execute", action="store_true", help="Unavailable until TODOs are implemented")
  args = parser.parse_args()
  try:
    config = load_config(args.config)
    if args.execute:
      execute_batch(config)
    for world in config["worlds"]:
      for baseline in config["baselines"]:
        for repetition in range(1, config["repetitions"] + 1):
          print(f"{world}/{baseline}/trial_{repetition:03d}: {' -> '.join(STAGES)}")
          for stage, command in TrialLifecycle(config, world, baseline, repetition).command_plan().items():
            print(f"  {stage}: {shlex.join(command)}")
  except (OSError, ValueError, yaml.YAMLError, NotImplementedError) as exc:
    parser.exit(2, f"benchmark_runner: {exc}\n")


if __name__ == "__main__":
  main()
