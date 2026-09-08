# Trial automation scaffold

This scaffold is separate from the existing `src/main.py` track follower.
Only configuration loading, matrix expansion, and dry-run output work today.
All ROS operations, monitoring, process cleanup, and result persistence are explicit
`NotImplementedError` hooks. No simulation is launched by the preview.

## Preview

From the `src/tbai_ros` repository directory, with Python 3 and PyYAML available:

```bash
python3 tbai_ros_benchmark/src/benchmark_runner.py --dry-run
python3 tbai_ros_benchmark/src/benchmark_runner.py --config tbai_ros_benchmark/config/benchmark.yaml
```

Both commands preview the same nine attempts by default. They do not require ROS,
create result directories, or check simulator readiness. `--execute` currently
exits with an implementation error before starting anything. Direct Python execution
is the scaffold entry point; catkin installation is not wired up yet.

## Files to fill in

| File | Responsibility |
| --- | --- |
| `config/benchmark.yaml` | Matrix, world poses/goals, timeouts, recording topics and termination rules |
| `src/benchmark_runner.py` | Configuration validation, batch ordering and try/finally orchestration |
| `src/trial_lifecycle.py` | Startup, readiness, reset, mapping, recording, motion, cleanup and persistence |
| `src/trial_monitor.py` | ROS subscriptions, simulation-time criteria, wall-time watchdogs and result contract |

## Suggested implementation order

1. Complete config validation. Validate launch/world files and reconcile supported
   worlds with `tbai_ros_utils/src/global_path_velocity_generator.py` and
   `tbai_ros_gazebo/reset_gazebo.sh`. Share goal configuration between generator and monitor.
2. Implement process ownership and cleanup together. Isolate ROS and Gazebo masters,
   redirect logs to files, track descendants, bound shutdown waits, and handle partial
   startup. Do not kill unrelated processes. Add SIGTERM handling as well as Ctrl+C.
3. Add a conditional `rviz` argument to each baseline's `anymal_d_perceptive.launch`.
   `gui:=false` currently only disables the Gazebo GUI.
4. Implement stack startup and readiness, then checked reset services and stable
   standing detection. Replace fixed sleeps with bounded state/service checks.
5. Start the shared `tbai_ros_gridmap/launch/elevation_mapping.launch` after reset;
   verify compatibility with each baseline and wait for fresh usable map data.
6. Add recording and motion activation using `run_experiment.sh` as a reference.
   Wait for recorder readiness before starting the measured interval.
7. Implement the monitor. Define upright goal completion, terrain-specific failure
   rules, stale-state detection and both simulation and wall deadlines. Keep robot
   failure separate from infrastructure error. Validate state frame/orientation units.
8. Save config, revision, timestamps, logs, bag and `TrialResult` per unique attempt;
   add summary CSV and resume semantics. Suggested layout:
   `results/<batch-id>/<world>/<baseline>/trial_001/`. Resolve `output_dir` relative
   to the config file and pass the resolved path into lifecycle hooks.
9. Replace `check_implementation()` with actual preflight checks once every hook is
   implemented. Run a single baseline before expanding the matrix.

The initial design fully restarts each trial. Reusing Gazebo, automatic retries,
parallel trials, timing instrumentation and random seed plumbing are later work.
Do not assume resetting pose/joints also clears controller, estimator or map history.

## Integration checks once implemented

- Three consecutive repetitions start at the configured pose with fresh maps.
- Success, fall, timeout, missing state and node crash produce distinct results.
- A stopped simulation clock triggers the wall watchdog.
- Ctrl+C during startup and motion finalizes available recordings and stops owned children.
- Cleanup failure stops the batch; no new trial starts with leftover processes.
- Completed attempts survive resume without overwriting their files.

Use sample timeouts and success rules as starting values for validation, not final
benchmark criteria. Unit tests can cover monitor decisions with synthetic state/time;
Gazebo integration checks are still needed for readiness and cleanup.
