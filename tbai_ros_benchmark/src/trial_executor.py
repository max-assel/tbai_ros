"""
Executes automated trials
"""
import argparse
from pathlib import Path
from trial_lifecycle import TrialLifecycle
import yaml

def load_config(path):
    path = path.resolve()
    with path.open() as stream:
        config = yaml.safe_load(stream)
    config["_config_path"] = str(path)
    return config

def execute_trial(config):
    results = []
    for world in config['worlds']:
        for controller in config['controllers']:
            for repetition in range(1, config['repetitions'] + 1):
                trial = TrialLifecycle(config, world, controller, repetition)
                print(f"Starting {world}/{controller}/trial_{repetition:03d}", flush=True)
                result = trial.execute()
                results.append(result)
                print(f"{world}/{controller}/trial_{repetition:03d}: {result.status}: {result.reason}", flush=True)
                print(f"  Trial duration: {result.duration_sim_sec:.3f} simulation seconds; "
                      f"recovery events: {len(result.recovery_events)}", flush=True)
                for event in result.recovery_events:
                    print(f"  Recovery: {event['kind']}; "
                          f"simulation time {event['start_sim_sec']:.3f}–{event['end_sim_sec']:.3f} s; "
                          f"duration {event['end_sim_sec'] - event['start_sim_sec']:.3f} s", flush=True)
                for error in result.cleanup_errors:
                    print(f"Cleanup error: {error}", flush=True)
                print(f"  Forward progress: {result.forward_progress_m:.3f} m", flush=True)
                if result.cleanup_errors or result.status not in config['execution']['continue_after']:
                    return results
    return results

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/benchmark_settings.yaml")
    args = parser.parse_args()
    
    try:
        config = load_config(args.config)
        results = execute_trial(config)

        if any(
            r.status in ("error", "interrupted") or r.cleanup_errors
            for r in results
        ):
            parser.exit(
                1,
                "Benchmark stopped; see trial results and logs.\n"
            )

    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
        parser.exit(2, f"benchmark_runner: {exc}\n")

if __name__ == "__main__":
  main()
