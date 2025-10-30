#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tbai_ros_msgs.msg import RbdState
import math

class GlobalPathVelocityGenerator:
    def __init__(self, world_name):
        self.world_name = world_name
        rospy.loginfo(f"GlobalPathVelocityGenerator initialized for world: {self.world_name}")
        # Additional initialization code here

        if (self.world_name == "balance_beam"):
            self.global_goal = [0.0, -5.70, 0.575]
        elif (self.world_name == "gap_stones"):
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

    def state_callback(self, msg):
        # Process the RbdState message and generate velocity commands
        rospy.loginfo(f"Received RbdState message: {msg}")
        # Velocity generation logic here
        current_pose = msg.rbd_state[:6]  # Assuming first 6 elements are orientation and position
        current_position = current_pose[3:6]

        dir_to_goal = [goal - current for goal, current in zip(self.global_goal, current_position)]
        
        # Normalize direction vector
        norm = math.sqrt(sum(coord ** 2 for coord in dir_to_goal))
        if norm > 0:
            dir_to_goal = [coord / norm for coord in dir_to_goal]
        
        # Generate velocity command
        velocity_command = Twist()
        velocity_command.linear.x = dir_to_goal[0]
        velocity_command.linear.y = dir_to_goal[1]
        velocity_command.angular.z = dir_to_goal[2]
        
        # Publish velocity command
        self.velocity_publisher.publish(velocity_command)

def main():
    rospy.init_node("global_path_velocity_generator", anonymous=True)

    # Get environment argument
    world_name = rospy.get_param("~world", "default")

    generator = GlobalPathVelocityGenerator(world_name)

    # Start the generator
    rospy.spin()


if __name__ == "__main__":
  main()
