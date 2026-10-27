#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros  # for .substitutions.FindPackageShare(...).find()

def generate_launch_description():
    # -------- Package paths --------
    # For RViz, we can use a substitution path (works fine as an argument to rviz2)
    rviz_default = PathJoinSubstitution([FindPackageShare('back_platform'), 'rviz', 'back_platform.rviz'])

    # For xacro, we need a real string path on disk
    back_platform_share = launch_ros.substitutions.FindPackageShare(
        package="back_platform"
    ).find("back_platform")
    xacro_path = os.path.join(back_platform_share, "xacro", "back_platform.xacro")

    # -------- xacro -> URDF --------
    # IMPORTANT: no trailing space after "xacro"
    robot_description_cmd = Command(["xacro ", xacro_path])

    # -------- Launch args --------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui if true'
    )
    rviz_switch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 if true'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_default,
        description='Full path to RViz config file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # -------- Nodes --------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_cmd},
            {"publish_frequency": 100.0},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(LaunchConfiguration('gui')),
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('gui')),
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        rviz_switch_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
