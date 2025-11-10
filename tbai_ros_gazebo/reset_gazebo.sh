#!/usr/bin/env bash


if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <envname>"
    exit 1
fi

source $(catkin locate)/devel/setup.bash

ENV_NAME=$1

ENV_X=0
ENV_Y=0
ENV_Z=0
ENV_QX=0
ENV_QY=0
ENV_QZ=0
ENV_QW=1


if [ "$ENV_NAME" == "balance_beam" ]; then
    echo "Setting up for balance_beam environment"
    ENV_X=0.0
    ENV_Y=-0.10
    ENV_Z=0.15
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=-0.7071
    ENV_QW=0.7071   # PI / 2
elif [ "$ENV_NAME" == "gap_stones" ]; then
    echo "Setting up for gap_stones environment"
    ENV_X=0.0
    ENV_Y=-0.50
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=-0.7071
    ENV_QW=0.7071   # PI / 2
elif [ "$ENV_NAME" == "ramp" ]; then
    echo "Setting up for ramp environment"
    ENV_X=0.0
    ENV_Y=-1.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=0.7071
    ENV_QW=0.7071   # PI / 2
elif [ "$ENV_NAME" == "ramped_balance_beam" ]; then
    echo "Setting up for ramped_balance_beam environment"
    ENV_X=0.0
    ENV_Y=0.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "ramped_stepping_stones" ]; then
    echo "Setting up for ramped_stepping_stones environment"
    ENV_X=-1.0
    ENV_Y=7.5
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
elif [ "$ENV_NAME" == "rubble" ]; then
    echo "Setting up for rubble environment"
    ENV_X=0.0
    ENV_Y=0.0
    ENV_Z=0.20
    ENV_QX=0
    ENV_QY=0
    ENV_QZ=1
    ENV_QW=0   # PI / 2
else
    echo "Unknown environment name: $ENV_NAME"
    exit 1
fi

## Pause Gazebo physics
rosservice call /gazebo/pause_physics

## Reset model state
rosservice call /gazebo/set_model_state "
            {
                model_state: 
                {
                    model_name: robot, 
                    pose: 
                    {
                        position: {x: "${ENV_X}", y: "${ENV_Y}", z: "${ENV_Z}"}, 
                        orientation: {x: "${ENV_QX}", y: "${ENV_QY}", z: "${ENV_QZ}", w: "${ENV_QW}"}
                    }, twist: 
                    {
                        linear: {x: 0.0, y: 0.0, z: 0.0}, 
                        angular: {x: 0.0, y: 0.0, z: 0.0}
                    }, 
                    reference_frame: world
                }
            }"

## Reset model configuration
rosservice call /gazebo/set_model_configuration "
            model_name: 'robot'
            urdf_param_name: 'robot_description'
            joint_names:
                - 'LF_KFE'
                - 'LH_KFE'
                - 'RF_KFE'
                - 'RH_KFE'
                - 'LF_HAA'
                - 'LH_HAA'
                - 'RF_HAA'
                - 'RH_HAA'
                - 'LF_HFE'
                - 'LH_HFE'
                - 'RF_HFE'
                - 'RH_HFE'
            joint_positions:
                - -2.60
                - 2.60
                - -2.60
                - 2.60
                - 0.0
                - 0.0
                - 0.0
                - 0.0
                - 1.50
                - -1.50
                - 1.50
                - -1.50"
## Set in ControlMode 1
# rosservice call /ControlMode 1
rostopic pub /anymal_d/change_controller std_msgs/String "data: STAND" --once

## Unause Gazebo physics
rosservice call /gazebo/unpause_physics

## Set in CheaterMode 1
# rosservice call /CheaterMode 1

sleep 1

# rostopic pub /anymal_d/change_controller std_msgs/String "data: WBC" --once
# rostopic pub /anymal_d/change_controller std_msgs/String "data: BOB" --once
rostopic pub /anymal_d/change_controller std_msgs/String "data: DTC" --once

sleep 1

rostopic pub /gait_command std_msgs/String "data: trot" --once

sleep 1

rosrun tbai_ros_utils global_path_velocity_generator.py __name:=global_path_velocity_generator _world:=$ENV_NAME