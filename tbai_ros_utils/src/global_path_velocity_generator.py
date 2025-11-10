#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tbai_ros_msgs.msg import RbdState
import numpy as np

class GlobalPathVelocityGenerator:
    def __init__(self, world_name):
        self.world_name = world_name
        rospy.loginfo(f"GlobalPathVelocityGenerator initialized for world: {self.world_name}")
        # Additional initialization code here

        if (self.world_name == "balance_beam"):
            self.global_goal = [0.0, -5.70, 0.575]
        elif (self.world_name == "gap_stones"):
            self.global_goal = [0.0, -4.50, 0.575]
        elif (self.world_name == "gap_stones_spaced"):
            self.global_goal = [0.0, -4.50, 0.575]            
        elif (self.world_name == "ramp"):
            self.global_goal = [0.0, 4.15, 0.785]
        elif (self.world_name == "ramped_balance_beam"):
            self.global_goal = [-8.50, 0.50, 0.575]
        elif (self.world_name == "ramped_stepping_stones"):
            self.global_goal = [-8.0, 7.50, 0.575]     
        elif (self.world_name == "rubble"):
            self.global_goal = [-5.50, 0.0, 0.575]                   
        else:
            rospy.logwarn(f"Unknown world name: {self.world_name}. Using default global goal.")
            raise Exception("[GlobalPathVelocityGenerator] Unknown world name")

        self.state_subscriber = rospy.Subscriber("anymal_d/state", RbdState, self.state_callback)
        self.velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.latest_state = None


    def state_callback(self, msg):
        self.latest_state = msg

    def send_velocity(self):
        state = self.latest_state
        if state is None:
            return

        # rospy.loginfo(f"Received RbdState message: {msg}")
        # Velocity generation logic here
        current_pose = state.rbd_state[:6]  # Assuming first 6 elements are orientation and position
        rospy.loginfo(f"Current Pose: {current_pose}")

        current_position = current_pose[3:6]

        rospy.loginfo(f"Current Position: {current_position}")

        rbt_to_goal_pos_odom_frame = np.array(self.global_goal) - np.array(current_position)

        rospy.loginfo(f"Global Goal: {self.global_goal}")

        # Normalize direction vector
        norm = np.linalg.norm(rbt_to_goal_pos_odom_frame[0:2]) # Only consider x and y for norm, Z we can assume is fine

        if (norm < 0.25):
            rospy.loginfo("Goal reached or very close to goal. Stopping.")
            velocity_command = Twist()
            velocity_command.linear.x = 0.0
            velocity_command.linear.y = 0.0
            velocity_command.angular.z = 0.0
            self.velocity_publisher.publish(velocity_command)
            return

        rbt_to_goal_dir_odom_frame = rbt_to_goal_pos_odom_frame / norm

        rospy.loginfo(f"Robot to Goal: {rbt_to_goal_pos_odom_frame}")

        # Transform into robot frame
        
        yaw = current_pose[2]
        rotation_matrix = np.array([
            [np.cos(-yaw), -np.sin(-yaw)],
            [np.sin(-yaw),  np.cos(-yaw)]
        ])
        
        dir_to_goal_robot_frame = rotation_matrix.dot(rbt_to_goal_dir_odom_frame[0:2]) # Only x and y

        rospy.loginfo(f"Direction to Goal in Robot Frame: {dir_to_goal_robot_frame}")

        lin_speed = 0.5  # m/s
        ang_speed = 0.5  # rad/s

        angular_error = np.arctan2(dir_to_goal_robot_frame[1], dir_to_goal_robot_frame[0])
        rospy.loginfo(f"Angular Error: {angular_error}")
        ang_command = ang_speed * angular_error

        # Generate velocity command
        velocity_command = Twist()
        velocity_command.linear.x = lin_speed * dir_to_goal_robot_frame[0]
        velocity_command.linear.y = lin_speed * dir_to_goal_robot_frame[1]
        velocity_command.angular.z = ang_command

        rospy.loginfo(f"Publishing Velocity Command: linear_x={velocity_command.linear.x}, linear_y={velocity_command.linear.y}, angular_z={velocity_command.angular.z}")
        
        # Publish velocity command
        self.velocity_publisher.publish(velocity_command)

def main():
    rospy.init_node("global_path_velocity_generator", anonymous=True)

    # Get environment argument
    world_name = rospy.get_param("~world", "default")

    generator = GlobalPathVelocityGenerator(world_name)

    # Start the generator
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        generator.send_velocity()
        rate.sleep()


if __name__ == "__main__":
  main()
