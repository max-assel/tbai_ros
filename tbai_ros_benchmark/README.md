# tbai_ros_benchmark

This ROS package servers the purpouse of providing utilities to test the different walking controllers implemented as part of this project.

## Benchmark map
![benchmark_map](https://github.com/lnotspotl/tbai/assets/82883398/b7d2ba54-859c-43c7-8429-f5e13a67c7ec)
![benchmark_map2](https://github.com/lnotspotl/tbai/assets/82883398/af20dfb1-f16f-4b5a-9917-839a60e971c8)

## Example - perceptive MPC



https://github.com/lnotspotl/tbai/assets/82883398/4c2b398c-04ed-4534-8e62-4b0e01aab8cf



https://github.com/lnotspotl/tbai/assets/82883398/1d07b358-187b-423f-b7b5-d3c9251ad7f5

## Example - context-aware controller
- Our context-aware controller switches between two of the implemented controllers, namely `rl_blind` and `mpc_perceptive`.
- In case a foot slip is detected, the context-aware controller changes the active controller from `mpc_perceptive` to `rl_blind`.
- Once a checkpoint has been reached, the active controller is changed to the `mpc_perceptive` controller again.

https://github.com/lnotspotl/tbai/assets/82883398/955b83fa-256b-492e-a959-1ddf97fbe860


## Automated trials

Keep `roscore` running in a separate terminal throughout the batch. This keeps
the executor's ROS node registered while each controller's Gazebo stack restarts.
From another terminal in the sourced catkin workspace, run:

```bash
python3 src/tbai_ros/tbai_ros_benchmark/src/trial_executor.py \
  --config src/tbai_ros/tbai_ros_benchmark/config/benchmark_settings.yaml
```

Each attempt writes process logs, `recording.bag`, and `result.json` beneath
`output_dir`. The runner waits for Gazebo/state readiness, stable standing, a
first elevation-map message, and recorder subscriptions to
all configured recording topics before starting motion. The map subscription is removed immediately after the first message; this gate
confirms publication, not terrain coverage or finite elevations.

The first nonzero motion command latches activation; the first state processed
after activation starts the simulation timer. Subsequent zero commands do not
suspend stuck detection. The monitor checks goal persistence, tilt,
low base height, course boundaries, route progress, and simulation
and wall deadlines. Route progress is projected onto the start-to-goal line,
matching the current straight goal-directed experiment runner. Recovery events
are posture-based estimates confirmed by resumed progress; they do not prove a
missed touchdown. Course boundary exits and low-height failures are recorded
separately, because the configuration does not contain support-surface geometry
needed to confirm a fall off terrain. Stuck detection uses time since the last configured minimum forward gain,
with constant memory; its window can accumulate during the initial grace period.
Success always requires upright posture, and unsuccessful robots inside the
goal tolerance remain subject to stuck detection. Thresholds need calibration against
labeled recordings before comparing controllers.

The normal `balance_beam` world is included in the trial list. Its start and goal
match the reset script and velocity generator. Between x = -1 and x = 1, an
optional boundary segment restricts the base position to y = [-0.25, 0.25],
matching the beam's width; the approach and exit use the wider course bounds.
This remains a course-boundary classification rather than proof of lost foot support.

The `rviz` setting controls RViz for MPC, DTC, and RL; set it to `false` for
trials without visualization. A Gazebo server exit shuts down its launch.

Cleanup stops process groups and allows rosbag to
finalize before stopping mapping and Gazebo. Errors and interruptions also
produce a result; cleanup errors stop the batch. During cleanup, additional
Ctrl+C presses are ignored until the bounded shutdown attempts finish. Roslaunch
gets enough time to escalate its own child processes before it is terminated.
`execution.continue_after`
controls which completed outcomes allow the next attempt.

Run the ROS-independent and mocked checks with:

```bash
python3 -m unittest discover -s src/tbai_ros/tbai_ros_benchmark/tests -v
```

### Monitor simplification line counts

For this simplification, excluding tests and documentation:

| File | Nonblank before | Nonblank after | Removed |
| --- | ---: | ---: | ---: |
| `src/trial_monitor.py` | 100 | 87 | 13 |
| `src/trial_lifecycle.py` | 254 | 252 | 2 |
| `config/benchmark_settings.yaml` | 193 | 175 | 18 |

The monitor is 13% shorter by nonblank lines (108 to 95 total lines).
The two Python files together shed 15 nonblank lines. Tilt and low-height
failures retain separate persistence timers and reasons. The inactive
`balance_beam` world configuration and segment handling were removed at that
time; both have since been restored to support trials in that environment.
