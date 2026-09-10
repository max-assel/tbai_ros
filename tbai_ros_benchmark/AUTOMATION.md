# Script-based trial automation skeleton

This replaces the previous in-progress implementation with wrappers around existing
commands. Robot reset, controller/gait selection and path following remain in
reset_gazebo.sh and run_experiment.sh. Existing main.py/helpers.py are unchanged.

Only configuration loading and command preview work. --execute fails explicitly
before launching anything. From src/tbai_ros, with Python 3.8+ and PyYAML:

    python3 tbai_ros_benchmark/src/benchmark_runner.py --dry-run

The preview is not a runnable shell script: readiness checks belong between the
commands, and long-running processes require ownership and cleanup.

## Files and fill-in order

1. config/benchmark.yaml: preserved world/controller selections and world settings.
   Tune deadlines/criteria and configure verified action, timing and recovery topics.
   Reset poses are expectations, not overrides passed to the script. Monitor goals
   must match the existing global_path_velocity_generator.py.
2. src/trial_lifecycle.py: implement ownership and cleanup together, then launch,
   readiness, calling reset_gazebo.sh, reset verification and mapping readiness.
   Invoke run_experiment.sh for motion. Use workspace cwd and sourced ROS environment
   because the scripts call catkin locate. Do not recreate their service calls.
3. src/trial_monitor.py: fixed maximum simulation duration plus early success/failure;
   record intermediate events and recovery already performed by existing controllers.
4. src/trial_metrics.py: summarize finalized bags and persist selected metrics.
5. src/benchmark_runner.py: wire sequential attempts, partial results and cleanup.

Each TODO section contains implementation suggestions. Execution hooks deliberately
raise NotImplementedError. Catkin installation is not wired up yet.

## Outcomes and failure handling

Final outcomes: success, failure, timeout, error, interrupted. Recoverable robot events
remain separate: success may include recoveries. Do not add recovery control here or
infer recovery success from a controller switch. Without an event source, recovery
metrics are unavailable. Keep incomplete episodes in the record.

Startup/reset/mapping/recorder problems are stage-tagged infrastructure errors.
Check reset exit status AND resulting state because intermediate service failures
may be masked by the script. Keep cleanup errors separate from the robot outcome;
stop the batch if owned processes remain. Always clean up partial startup too.

Use simulation time for duration/time-to-goal and monotonic wall time for watchdogs.
Recovery does not restart the duration clock. Define tie precedence and sustained,
terrain-specific termination criteria; current thresholds are provisional.
RViz currently launches unconditionally: add optional launch support before passing
an rviz argument. gui:=false controls only Gazebo.

## Selected data and outputs

- Outcome/reason, intermediate failures, recovery start/end/outcome.
- Trial simulation/wall duration and confirmed time to goal.
- Timestamped robot state and high-level/low-level actions in rosbag.
- XY path length and final XY goal distance in a verified common frame.
- MPC/WBC/policy runtime count, mean, median, p95, p99 and maximum.

Distance calculations exclude reset/cleanup. Document filtering, gaps, frames and
sample alignment; do not bridge teleports. Missing data is null with a reason, not
zero. Publication frequency is not runtime: use measured call durations, defining
units, warmup exclusion, percentile method and GPU completion timing if relevant.

Each unique results/<batch>/<world>/<baseline>/trial_001 directory should contain:
metadata.yaml, result.json, events.jsonl, runtime_summary.json, recording.bag and
per-process logs. Append one batch summary.csv row per attempt. Record/arm monitor
before motion, finalize bag before analysis, preserve raw artifacts on analysis
failure. Automatic retries, resume, stack reuse and parallel trials are later work.

## Validation after implementation

Start with three trials for one controller. Exercise early success, terminal failure,
intermediate recovery, timeout, reset failure, stale state, clock stall, recorder
crash and Ctrl+C during startup/motion. Verify partial results, bag finalization and
no owned children left; never advance after failed cleanup. Check metrics against a
known recording before expanding to more worlds/controllers.
