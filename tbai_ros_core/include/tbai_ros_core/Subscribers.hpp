#pragma once

#include <algorithm>
#include <atomic>
#include <string>
#include <vector>

#include <tbai_ros_msgs/msg/rbd_state.hpp>
#include <tbai_ros_msgs/msg/robot_state.hpp>
// #include <ros/callback_queue.h>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_estim/muse/MuseEstimator.hpp>

namespace tbai {

class RosStateSubscriber : public tbai::StateSubscriber 
{
   public:
    RosStateSubscriber(const rclcpp::Node::SharedPtr & node, const std::string &stateTopic);

    void waitTillInitialized() override;

    State getLatestState() override;

   private:
    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::msg::RbdState::SharedPtr msg);

    /** Shared pointer to the latest state message */
    tbai_ros_msgs::msg::RbdState::SharedPtr stateMessage_;

    /** State message subscriber */
    rclcpp::Subscription<tbai_ros_msgs::msg::RbdState>::SharedPtr stateSubscriber_;

    rclcpp::Node::SharedPtr node_;
};

class RosChangeControllerSubscriber : public ChangeControllerSubscriber 
{
   public:
    RosChangeControllerSubscriber(const rclcpp::Node::SharedPtr & node, const std::string &topic) 
    {
        // controllerSubscriber_ = nh.subscribe(topic, 1, &RosChangeControllerSubscriber::controllerCallback, this);
        controllerSubscriber_ = node->create_subscription<std_msgs::msg::String>(topic, 1, std::bind(&RosChangeControllerSubscriber::controllerCallback, this, std::placeholders::_1));
    }

    void triggerCallbacks() override {
        if (latestControllerType_.empty()) {
            return;
        }
        callbackFunction_(latestControllerType_);
        latestControllerType_.clear();
    }

   private:
    void controllerCallback(const std_msgs::msg::String::SharedPtr msg) { latestControllerType_ = msg->data; }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controllerSubscriber_;
    std::string latestControllerType_;
};

class MuseRosStateSubscriber : public tbai::ThreadedStateSubscriber 
{
   public:
    MuseRosStateSubscriber(const rclcpp::Node::SharedPtr & nodetemp, const std::string &stateTopic, const std::string &urdf = "");
    ~MuseRosStateSubscriber() override { stopThreads(); }
    void waitTillInitialized() override;

    void startThreads() override;
    void stopThreads() override;

   private:
    /** State message subscriber */
    rclcpp::Subscription<tbai_ros_msgs::msg::RobotState>::SharedPtr stateSubscriber_;

    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::msg::RobotState::SharedPtr msg);

    void threadFunction();
    std::thread stateThread_;

    std::atomic_bool isRunning_ = false;
    std::atomic_bool isInitialized_ = false;

    // ros::CallbackQueue thisQueue_;
    std::unique_ptr<tbai::muse::MuseEstimator> estimator_;

    std::shared_ptr<spdlog::logger> logger_;
    rclcpp::Time lastStateTime_;
    bool firstState_ = true;
    scalar_t lastYaw_ = 0.0;

    rclcpp::Node::SharedPtr node_;
};

class InekfRosStateSubscriber : public tbai::ThreadedStateSubscriber 
{
   public:
    InekfRosStateSubscriber(const rclcpp::Node::SharedPtr & nodetemp, const std::string &stateTopic, const std::string &urdf = "");
    ~InekfRosStateSubscriber() override { stopThreads(); }
    void waitTillInitialized() override;

    void startThreads() override;
    void stopThreads() override;

   private:
    /** State message subscriber */
    rclcpp::Subscription<tbai_ros_msgs::msg::RobotState>::SharedPtr stateSubscriber_;

    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::msg::RobotState::SharedPtr msg);

    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }

    bool enable_ = false;

    void threadFunction();
    std::thread stateThread_;

    std::atomic_bool isRunning_ = false;
    std::atomic_bool isInitialized_ = false;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    rclcpp::CallbackGroup::SharedPtr callbackGroup_;
    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    std::shared_ptr<spdlog::logger> logger_;
    rclcpp::Time lastStateTime_;
    bool firstState_ = true;
    scalar_t lastYaw_ = 0.0;

    rclcpp::Node::SharedPtr node_;
};

}  // namespace tbai
