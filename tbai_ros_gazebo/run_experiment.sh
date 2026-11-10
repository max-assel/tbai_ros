#!/usr/bin/env bash

if [[ $# -ne 2 && ! ( $# -eq 3 && $3 == --no-controller ) ]]; then
    echo "Usage: $0 <envname> <baseline> [--no-controller]"
    exit 1
fi

source $(catkin locate)/devel/setup.bash


ENV_NAME=$1
BASELINE=$2

if [[ "$BASELINE" != MPC && "$BASELINE" != RL && "$BASELINE" != DTC ]]; then
    echo "Unknown baseline: $BASELINE"
    exit 1
fi

# Automated trials select the controller and gait in their persistent executor.
if [ "${3:-}" != "--no-controller" ]; then
    if [ "$BASELINE" == "MPC" ]; then
        GAIT_NAME=trot
        if [ "$(rosparam get /robot_name)" = "go2" ]; then
            GAIT_NAME=go2_trot
        fi
        rostopic pub /anymal_d/change_controller std_msgs/String "data: WBC" --once
        sleep 1
        rostopic pub /gait_command std_msgs/String "data: $GAIT_NAME" --once
    elif [ "$BASELINE" == "RL" ]; then
        rostopic pub /anymal_d/change_controller std_msgs/String "data: BOB" --once
    elif [ "$BASELINE" == "DTC" ]; then
        rostopic pub /anymal_d/change_controller std_msgs/String "data: DTC" --once
        sleep 1
        rostopic pub /gait_command std_msgs/String "data: trot" --once
    fi
    sleep 1
fi

rosrun tbai_ros_utils global_path_velocity_generator.py __name:=global_path_velocity_generator _world:=$ENV_NAME _planner:=$BASELINE
