#pragma once

#include <tbai_core/control/Publishers.hpp>
#include <tbai_ros_msgs/JointCommandArray.h>

namespace tbai {

class RosCommandPublisher : public tbai::CommandPublisher {
   public:
    RosCommandPublisher(const rclcpp::Node::SharedPtr &node, const std::string &commandTopic) 
    {
        // commandPublisher_ = nh.advertise<tbai_ros_msgs::JointCommandArray>(commandTopic, 1);
        commandPublisher_ = node->create_publisher<tbai_ros_msgs::JointCommandArray>(commandTopic, 1);
    }

    virtual void publish(std::vector<MotorCommand> commands) override 
    {
        // Message to be sent to the motor controller
        tbai_ros_msgs::JointCommandArray commandArray;
        commandArray.joint_commands.resize(commands.size());

        // Populate command array
        for (size_t i = 0; i < commands.size(); ++i) {
            tbai_ros_msgs::JointCommand command;
            command.joint_name = commands[i].joint_name;
            command.desired_position = commands[i].desired_position;
            command.desired_velocity = commands[i].desired_velocity;
            command.kp = commands[i].kp;
            command.kd = commands[i].kd;
            command.torque_ff = commands[i].torque_ff;
            commandArray.joint_commands[i] = command;
        }

        // Send message
        commandPublisher_.publish(commandArray);
    }

   private:
    // ros::Publisher commandPublisher_;
    rclcpp::Publisher<tbai_ros_msgs::JointCommandArray>::SharedPtr commandPublisher_;
};

}  // namespace tbai