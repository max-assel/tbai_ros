#!/usr/bin/env python3
"""Preview or execute sequential benchmark trials using the existing robot scripts."""

import argparse
from pathlib import Path
import shlex

import yaml

from trial_lifecycle import STAGES, TrialLifecycle


def load_config(path):
  """Read the configuration and resolve script paths relative to its file."""
  path = path.resolve()
  with path.open() as stream:
    config = yaml.safe_load(stream)
  for key in ("reset", "run"):
    config["scripts"][key] = str((path.parent / config["scripts"][key]).resolve())
  config["_config_path"] = str(path)
  return config


def execute_batch(config):
  """Run the matrix sequentially; never advance after uncertain cleanup."""
  results = []
  for world in config['worlds']:
    for baseline in config['baselines']:
      for repetition in range(1, config['repetitions'] + 1):
        trial = TrialLifecycle(config, world, baseline, repetition)
        result = trial.execute()
        results.append(result)
        print(f"{world}/{baseline}/trial_{repetition:03d}: {result.status}: {result.reason}", flush=True)
        if result.cleanup_errors or result.status not in config['execution']['continue_after']:
          return results
  return results


def main():
  """Default to a preview; execute only when requested."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/benchmark.yaml")
  mode = parser.add_mutually_exclusive_group()
  mode.add_argument("--dry-run", action="store_true", help="Preview only (default)")
  mode.add_argument("--execute", action="store_true", help="Run the configured trials sequentially")
  args = parser.parse_args()
  try:
    config = load_config(args.config)
    if args.execute:
      results = execute_batch(config)
      if any(r.status in ("error", "interrupted") or r.cleanup_errors for r in results):
        parser.exit(1, "Benchmark stopped; see trial results and logs.\n")
      return
    for world in config["worlds"]:
      for baseline in config["baselines"]:
        for repetition in range(1, config["repetitions"] + 1):
          print(f"{world}/{baseline}/trial_{repetition:03d}: {' -> '.join(STAGES)}")
          for stage, command in TrialLifecycle(config, world, baseline, repetition).command_plan().items():
            print(f"  {stage}: {shlex.join(command)}")
  except (OSError, ValueError, yaml.YAMLError) as exc:
    parser.exit(2, f"benchmark_runner: {exc}\n")


if __name__ == "__main__":
  main()
