# INSTRUCTIONS TO RUN

## GRID MAP MPC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=true world:=ramp

rosbag record -a

rosbag record /anymal_d/change_controller /anymal_d/command /anymal_d/state /anymal_mode_schedule /anymal_mpc_gait_schedule /anymal_mpc_mode_schedule /anymal_mpc_observation /anymal_mpc_policy /anymal_mpc_scheduled_mode_schedule /anymal_mpc_target /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points /camera/depth/points_downsampled /camera/parameter_descriptions /camera/parameter_updates /camera/rgb/camera_info /camera/rgb/image_raw /camera/rgb/image_raw/compressed /camera/rgb/image_raw/compressed/parameter_descriptions /camera/rgb/image_raw/compressed/parameter_updates /camera/rgb/image_raw/compressedDepth /camera/rgb/image_raw/compressedDepth/parameter_descriptions /camera/rgb/image_raw/compressedDepth/parameter_updates /camera/rgb/image_raw/theora /camera/rgb/image_raw/theora/parameter_descriptions /camera/rgb/image_raw/theora/parameter_updates /clicked_point /clock /cmd_vel /contact_points /convex_plane_decomposition_ros/boundaries /convex_plane_decomposition_ros/filtered_map /convex_plane_decomposition_ros/insets /convex_plane_decomposition_ros/planar_terrain /convex_plane_decomposition_ros/signed_distance_field /elevation_mapping/elevation_map /elevation_mapping/elevation_map_raw /elevation_mapping/visibility_cleanup_map /gait_command /gazebo/link_states /gazebo/model_states /gazebo/parameter_descriptions /gazebo/parameter_updates /gazebo/performance_metrics /gazebo/set_link_state /gazebo/set_model_state /initialpose /left_camera/depth/points_downsampled /lf_foot_contact /lh_foot_contact /local_terrain /move_base_simple/goal /ocs2_anymal/currentCollisionSpheres /ocs2_anymal/currentFeetPoses /ocs2_anymal/currentState /ocs2_anymal/desiredAngVelTrajectory /ocs2_anymal/desiredBaseTrajectory /ocs2_anymal/desiredFeetTrajectory/LF /ocs2_anymal/desiredFeetTrajectory/LH /ocs2_anymal/desiredFeetTrajectory/RF /ocs2_anymal/desiredFeetTrajectory/RH /ocs2_anymal/desiredFeetVelTrajectory/LF /ocs2_anymal/desiredFeetVelTrajectory/LH /ocs2_anymal/desiredFeetVelTrajectory/RF /ocs2_anymal/desiredFeetVelTrajectory/RH /ocs2_anymal/desiredPoseTrajectory /ocs2_anymal/desiredVelTrajectory /ocs2_anymal/localTerrain /ocs2_anymal/optimizedPoseTrajectory /ocs2_anymal/optimizedStateTrajectory /ocs2_anymal/swing_planner/nominalFootholds_LF /ocs2_anymal/swing_planner/nominalFootholds_LH /ocs2_anymal/swing_planner/nominalFootholds_RF /ocs2_anymal/swing_planner/nominalFootholds_RH /ocs2_anymal/swing_planner/trajectory_LF /ocs2_anymal/swing_planner/trajectory_LH /ocs2_anymal/swing_planner/trajectory_RF /ocs2_anymal/swing_planner/trajectory_RH /pcl_manager/bond /rear_camera/depth/points_downsampled /rf_foot_contact /rh_foot_contact /right_camera/depth/points_downsampled /rosout /rosout_agg /tf /tf_static /voxel_grid/parameter_descriptions /voxel_grid/parameter_updates /voxel_grid1/parameter_descriptions /voxel_grid1/parameter_updates /voxel_grid2/parameter_descriptions /voxel_grid2/parameter_updates /voxel_grid3/parameter_descriptions /voxel_grid3/parameter_updates

## GRID MAP RL
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=true world:=ramp

rosbag record -a


## GRID MAP DTC
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_dtc simple.launch gui:=true world:=ramp


# RAMP ENVIRONMENT

Can just run straight up and see what we got