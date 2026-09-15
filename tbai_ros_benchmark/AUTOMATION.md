# Script-based trial automation

Run from the sourced ROS 1 catkin workspace with Python 3, PyYAML and psutil.
The observer also uses the workspace's rospy and ROS message packages;
bag analysis uses rosbag. Preview is the default and does not import ROS:

```bash
python3 src/tbai_ros/tbai_ros_benchmark/src/benchmark_runner.py --dry-run
python3 src/tbai_ros/tbai_ros_benchmark/src/benchmark_runner.py --execute
```

Use `--config /path/to/benchmark.yaml` to select another configuration. Preparation
assumes the environment, scripts and world settings are correct. It does not
preflight packages or compare generator goals. Keep configured goals consistent
with `global_path_velocity_generator.py`.

## Trial sequence

1. Create a unique attempt directory under the batch, snapshot configuration, and
   select separate localhost ROS and Gazebo master ports.
2. Start a ROS master and the observer, then launch the selected controller stack.
   Wait for advancing clock, fresh state and Gazebo reset services.
3. Run the existing `reset_gazebo.sh` with `bash -e`. Confirm the configured start
   XY and sustained standing posture/low speed using fresh state.
4. Start the shared elevation mapping launch.
5. Start rosbag. Mapping coverage and incoming recording streams are not checked
   before continuing.
6. Arm the monitor, then invoke `run_experiment.sh` with `bash -e`. The first
   nonzero velocity request starts measurement.
7. Stop on sustained goal arrival, fall, course departure, no progress, or duration
   limit. Stale state, stalled/regressing clock and process exits are errors.
8. Stop the motion script and its descendants, publish zero velocity, finalize
   rosbag, and stop mapping, controllers, observer and master. Escalate SIGINT to
   SIGTERM and SIGKILL using the configured wall deadlines.
9. Save the outcome, analyze the finalized bag, and append one batch summary row.
   Continue sequentially only for configured outcomes with successful cleanup.

Robot reset, controller/gait selection and path following remain in the existing
scripts. `main.py` and `helpers.py` are unchanged. `rviz` controls RViz and
`gazebo_gui` controls the Gazebo window independently.

## Implementation

- `trial_lifecycle.py`: process ownership, readiness, script invocation, cleanup
  and per-attempt execution. Children inherit the attempt's environment and cwd.
  Process identities and inherited attempt tags identify owned descendants even
  if they start new sessions or are reparented. No global kill commands are used.
- `trial_ros.py`: one observer subprocess per attempt, avoiding rospy reinitialization
  across masters. It writes atomic snapshots for the parent and handles zero velocity.
- `trial_monitor.py`: evaluates snapshots at `readiness.poll_wall_sec`. Simulation
  time governs sustained conditions and measured duration; monotonic wall time
  governs startup, freshness and process shutdown. Precedence is error, failure,
  success, timeout. Failure candidates generate onset/clearance events.
- `trial_metrics.py`: analyzes native state timestamps within motion start through
  termination, leaving actions at their original timestamps in the raw bag.
- `benchmark_runner.py`: executes the configured world/baseline/repetition matrix.

The implementation targets the supplied configuration: sequential fresh stacks,
world-frame RbdState, XY trajectory, uncompressed bags and no CSV stream export.
Recovery remains unavailable because there is no verified event source; candidate
clearances and controller switches are not counted as recoveries. No recovery
control or runtime instrumentation is added. Configured runtime topics are recorded
and summarized; the default null sources yield unavailable statistics.

## Outputs and failure handling

Each attempt uses `results/<batch>/<world>/<baseline>/trial_001_<unique>/` and stores
`metadata.yaml`, `result.json`, `events.jsonl`, `runtime_summary.json`,
`recording.bag`, observer snapshots and process logs. A shared loaded config shares
one batch directory; loading a fresh config starts a new batch. Git is not queried.

The result records infrastructure stage/reason separately from cleanup errors.
Ctrl+C/SIGTERM triggers cleanup and persistence. Another interrupt during cleanup
is ignored so it cannot abandon owned processes. Any cleanup error stops the batch.
An unfinalized `.bag.active` is preserved rather than analyzed. Analysis errors are
saved without replacing the original trial outcome or deleting the raw bag.

XY distance sums consecutive valid state samples. Duplicate timestamps keep the
first sample; gaps, time regressions and implausible jumps make distance unavailable.
Reset and cleanup samples are excluded. Missing measurements are null with reasons.
Runtime summaries use measured durations, convert to milliseconds and report count,
mean, median, p95, p99 and maximum with linear percentile interpolation.

## Validation

Offline checks:

```bash
python3 -m unittest discover -s src/tbai_ros/tbai_ros_benchmark/tests -v
```

These exercise termination rules, candidate events, timestamp windows, runtime
summaries, partial failures, interruption persistence and actual descendant cleanup.
ROS/Gazebo integration still needs a live run. Begin with one world and baseline;
verify reset, map coverage, goal arrival, bag finalization and no remaining children
before expanding the matrix. Terrain thresholds in the configuration need tuning.
