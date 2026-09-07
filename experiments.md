# INSTRUCTIONS TO RUN
Need to open four terminals to run.

Different baselines: `MPC`, `RL`, `DTC`

Different environments: `balance_beam`, `pegboard`, `ramp_10`, `ramped_balance_beam`, `ramped_stepping_stones`, `rubble`, `side_stones`, `sparse_stones`, `stairs`

# Commands
## MPC 
```
# Terminal 1 - Gazebo
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=<ENV_NAME>
```

```
# Terminal 2 - Reset Gazebo for experiment
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh <ENV_NAME> MPC
```

```
# Terminal 3 - Elevation mapping
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
```

```
# Terminal 4 - Run experiment
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh <ENV_NAME> MPC
```

## RL
```
# Terminal 1 - Gazebo
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=<ENV_NAME>
```

```
# Terminal 2 - Reset Gazebo for experiment
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh <ENV_NAME> RL
```

```
# Terminal 3 - Elevation mapping
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob elevation_mapping.launch 
```

```
# Terminal 4 - Run experiment
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh <ENV_NAME> RL
```

## DTC
```
# Terminal 1 - Gazebo
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc anymal_d_perceptive.launch gui:=false world:=<ENV_NAME>
```

```
# Terminal 2 - Reset Gazebo for experiment
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh <ENV_NAME> DTC
```

```
# Terminal 3 - Elevation mapping
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc elevation_mapping.launch 
```

```
# Terminal 4 - Run experiment
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh <ENV_NAME> DTC
```
