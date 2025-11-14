# INSTRUCTIONS TO RUN

## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=true world:=ramp

rosbag record -a

rosbag record /anymal_d/change_controller /anymal_d/command /anymal_d/state /anymal_mode_schedule /anymal_mpc_gait_schedule /anymal_mpc_mode_schedule /anymal_mpc_observation /anymal_mpc_policy /anymal_mpc_scheduled_mode_schedule /anymal_mpc_target /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points /camera/depth/points_downsampled /camera/parameter_descriptions /camera/parameter_updates /camera/rgb/camera_info /camera/rgb/image_raw /camera/rgb/image_raw/compressed /camera/rgb/image_raw/compressed/parameter_descriptions /camera/rgb/image_raw/compressed/parameter_updates /camera/rgb/image_raw/compressedDepth /camera/rgb/image_raw/compressedDepth/parameter_descriptions /camera/rgb/image_raw/compressedDepth/parameter_updates /camera/rgb/image_raw/theora /camera/rgb/image_raw/theora/parameter_descriptions /camera/rgb/image_raw/theora/parameter_updates /clicked_point /clock /cmd_vel /contact_points /convex_plane_decomposition_ros/boundaries /convex_plane_decomposition_ros/filtered_map /convex_plane_decomposition_ros/insets /convex_plane_decomposition_ros/planar_terrain /convex_plane_decomposition_ros/signed_distance_field /elevation_mapping/elevation_map /elevation_mapping/elevation_map_raw /elevation_mapping/visibility_cleanup_map /gait_command /gazebo/link_states /gazebo/model_states /gazebo/parameter_descriptions /gazebo/parameter_updates /gazebo/performance_metrics /gazebo/set_link_state /gazebo/set_model_state /initialpose /left_camera/depth/points_downsampled /lf_foot_contact /lh_foot_contact /local_terrain /move_base_simple/goal /ocs2_anymal/currentCollisionSpheres /ocs2_anymal/currentFeetPoses /ocs2_anymal/currentState /ocs2_anymal/desiredAngVelTrajectory /ocs2_anymal/desiredBaseTrajectory /ocs2_anymal/desiredFeetTrajectory/LF /ocs2_anymal/desiredFeetTrajectory/LH /ocs2_anymal/desiredFeetTrajectory/RF /ocs2_anymal/desiredFeetTrajectory/RH /ocs2_anymal/desiredFeetVelTrajectory/LF /ocs2_anymal/desiredFeetVelTrajectory/LH /ocs2_anymal/desiredFeetVelTrajectory/RF /ocs2_anymal/desiredFeetVelTrajectory/RH /ocs2_anymal/desiredPoseTrajectory /ocs2_anymal/desiredVelTrajectory /ocs2_anymal/localTerrain /ocs2_anymal/optimizedPoseTrajectory /ocs2_anymal/optimizedStateTrajectory /ocs2_anymal/swing_planner/nominalFootholds_LF /ocs2_anymal/swing_planner/nominalFootholds_LH /ocs2_anymal/swing_planner/nominalFootholds_RF /ocs2_anymal/swing_planner/nominalFootholds_RH /ocs2_anymal/swing_planner/trajectory_LF /ocs2_anymal/swing_planner/trajectory_LH /ocs2_anymal/swing_planner/trajectory_RF /ocs2_anymal/swing_planner/trajectory_RH /pcl_manager/bond /rear_camera/depth/points_downsampled /rf_foot_contact /rh_foot_contact /right_camera/depth/points_downsampled /rosout /rosout_agg /tf /tf_static /voxel_grid/parameter_descriptions /voxel_grid/parameter_updates /voxel_grid1/parameter_descriptions /voxel_grid1/parameter_updates /voxel_grid2/parameter_descriptions /voxel_grid2/parameter_updates /voxel_grid3/parameter_descriptions /voxel_grid3/parameter_updates

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

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh balance_beam DTC

# Pegboard Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=pegboard
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh pegboard MPC

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=pegboard
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh pegboard RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=pegboard
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh pegboard DTC


# Rubble Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=rubble
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh rubble MPC

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=rubble
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh rubble RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=rubble
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh rubble DTC


# Ramp (10) Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=ramp_10
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramp_10 MPC

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=ramp_10
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramp_10 RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=ramp_10
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramp_10 DTC

# Stairs Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=stairs
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh stairs MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh stairs MPC
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=stairs
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh stairs RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=stairs
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh stairs DTC

# Ramped Balance Beam Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=ramped_balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_balance_beam MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh ramped_balance_beam MPC
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=ramped_balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_balance_beam RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=ramped_balance_beam
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_balance_beam DTC

# Ramped Stepping Stones Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=ramped_stepping_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_stepping_stones MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh ramped_stepping_stones MPC
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=ramped_stepping_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_stepping_stones RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=ramped_stepping_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh ramped_stepping_stones DTC

# Sparse Stones Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=sparse_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh sparse_stones MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh sparse_stones MPC
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=sparse_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh sparse_stones RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh sparse_stones DTC

# Side Stones Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=false world:=side_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh side_stones MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc elevation_mapping.launch 
source $(catkin locate)/devel/setup.bash && ./run_experiment.sh side_stones MPC
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=false world:=side_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh side_stones RL

## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=false world:=side_stones
source $(catkin locate)/devel/setup.bash && ./reset_gazebo.sh side_stones DTC
























# Gap Stones Spaced Environment
## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=true world:=gap_stones_spaced
## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=true world:=gap_stones_spaced
## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=true world:=gap_stones_spaced

Need to set some guidelines. 
- How long to let a trial run?

Worth mentioning that stones are uniform here, so if the robot can time it luckily, can get across some or all. If we did random or added some noise or left some out, these would probably fail more.

# RAMP ENVIRONMENT

Can just run straight up and see what we got

Running with Gazebo GUI + RViz, I don't think this is the difference between a success and a failure for these methods though

TODO: get what angle the ramp is at to report in paper