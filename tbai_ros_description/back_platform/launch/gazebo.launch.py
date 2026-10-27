#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package and file paths
    pkg_share = FindPackageShare('back_platform')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'back_platform.urdf'])
    
    # Gazebo package
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='SDF world file'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='x position of robot spawn'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='y position of robot spawn'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='z position of robot spawn'
    )
    
    # Robot description
    robot_description = Command(['cat ', urdf_file])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'back_platform',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose')
        ]
    )
    
    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])