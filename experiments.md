# INSTRUCTIONS TO RUN

## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=true world:=ramp

### Plotting
- Best way to get average MPC rate? Can just echo observation, that is published whenever /ocs2_anymal_observation is updated which is throttled at desired MPC rate
- Same for WBC, could just average /anymal_d/command rate? Not entirely accurate, better would be to record times before/after call inside the code and then echo that

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=true world:=ramp

rosbag record -a


## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=true world:=ramp

contact_flags in state are not updated. Not sure why. [FIXED]

# Balance Beam Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh balance_beam MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh balance_beam MPC

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh balance_beam RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh balance_beam RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc anymal_d_perceptive.launch gui:=false world:=balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh balance_beam DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh balance_beam DTC

Need to set some guidelines. 
- How long to let a trial run?

Worth mentioning that stones are uniform here, so if the robot can time it luckily, can get across some or all. If we did random or added some noise or left some out, these would probably fail more.

# RAMP ENVIRONMENT

Can just run straight up and see what we got

Running with Gazebo GUI + RViz, I don't think this is the difference between a success and a failure for these methods though

TODO: get what angle the ramp is at to report in paper