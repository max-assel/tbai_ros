#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package and file paths
    pkg_share = FindPackageShare('back_platform')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'back_platform.urdf'])
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher (no GUI to avoid graphics issues)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])