# INSTRUCTIONS TO RUN
Need to open four terminals to run.

Different baselines: `MPC`, `RL`, `DTC`

Different environments: `balance_beam`, `pegboard`, `ramp_10`, `ramped_balance_beam`, `ramped_stepping_stones`, `rubble`, `side_stones`, `sparse_stones`, `stairs`

# Commands
```
# Terminal 1 - Gazebo
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=balance_beam
```

```
# Terminal 2 - Reset Gazebo for experiment
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh balance_beam MPC
```

```
# Terminal 3 - Elevation mapping
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
```

```
# Terminal 4 - Run experiment
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh balance_beam MPC
```
