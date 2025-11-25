#!/usr/bin/env bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <envname> <baseline>"
    exit 1
fi

source $(catkin locate)/devel/setup.bash


ENV_NAME=$1
BASELINE=$2

if [ "$BASELINE" == "MPC" ]; then
    rostopic pub /anymal_d/change_controller std_msgs/String "data: WBC" --once
    sleep 1
    rostopic pub /gait_command std_msgs/String "data: trot" --once
elif [ "$BASELINE" == "RL" ]; then
    rostopic pub /anymal_d/change_controller std_msgs/String "data: BOB" --once
elif [ "$BASELINE" == "DTC" ]; then
    rostopic pub /anymal_d/change_controller std_msgs/String "data: DTC" --once
    sleep 1
    rostopic pub /gait_command std_msgs/String "data: trot" --once    
else
    echo "Unknown baseline: $BASELINE"
    exit 1
fi

sleep 1

rosrun tbai_ros_utils global_path_velocity_generator.py __name:=global_path_velocity_generator _world:=$ENV_NAME _planner:=$BASELINE