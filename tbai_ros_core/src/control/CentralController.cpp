#include "tbai_ros_core/control/CentralController.hpp"

#include <ros/ros.h>
#include <tbai_ros_core/Utils.hpp>
#include <tbai_ros_msgs/JointCommandArray.h>

#include <tbai_ros_core/control/CommandPublisher.hpp>

namespace tbai {

using namespace tbai::core;

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosCentralController::RosCentralController(ros::NodeHandle &nh, const std::string &stateTopic,
                                     const std::string &commandTopic, const std::string &changeControllerTopic)
    : loopRate_(1), CentralController() {
    initTime_ = tbai::core::getEpochStart();

    stateSubscriberPtr_ = std::shared_ptr<StateSubscriber>(new RosStateSubscriber(nh, stateTopic));
    changeControllerSubscriber_ = nh.subscribe(changeControllerTopic, 1, &RosCentralController::changeControllerCallback, this);
    commandPublisher_ = std::unique_ptr<CommandPublisher>(new RosCommandPublisher(nh, commandTopic));
    fallbackControllerType_ = "SIT";
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosCentralController::RosCentralController(ros::NodeHandle &nh, const std::string &configParam)
    : loopRate_(1), CentralController() {
    auto config = YamlConfig::fromRosParam(configParam);
    auto stateTopic = config.get<std::string>("state_topic");
    auto commandTopic = config.get<std::string>("command_topic");
    auto changeControllerTopic = config.get<std::string>("change_controller_topic");

    stateSubscriberPtr_ = std::shared_ptr<StateSubscriber>(new RosStateSubscriber(nh, stateTopic));
    changeControllerSubscriber_ = nh.subscribe(changeControllerTopic, 1, &RosCentralController::changeControllerCallback, this);
    commandPublisher_ = std::unique_ptr<CommandPublisher>(new RosCommandPublisher(nh, commandTopic));
    fallbackControllerType_ = "SIT";
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosCentralController::start() {
    std::cerr << "[DEBUG]Starting controller loop" << std::endl;

    // Make sure there is an active controller
    if (activeController_ == nullptr) {
        std::cerr << "[DEBUG]No active controller found" << std::endl;
        ROS_ERROR("No active controller found");
        return;
    }
    std::cerr << "[DEBUG]Active controller found" << std::endl;

    // Check if fallback controller is available
    containsFallbackController_ = checkForFallbackController();
    if (!containsFallbackController_) {
        ROS_WARN("Fallback controller not found, no stability checking will be performed.");
    }

    // Wait for initial state message
    std::cerr << "[DEBUG]Waiting for initial state message" << std::endl;
    stateSubscriberPtr_->waitTillInitialized();

    if (ros::ok()) {
        // Main loop rate
        loopRate_ = ros::Rate(activeController_->getRate());
    }

    std::cerr << "[DEBUG]Initial state message received" << std::endl;

    scalar_t lastTime = getCurrentTime();
    while (ros::ok()) {

        // Keep track of time for stats
        auto t1 = std::chrono::high_resolution_clock::now();

        // Spin once to allow ROS run callbacks
        ros::spinOnce();

        // Check stability and switch to fallback controller if necessary
        if (containsFallbackController_ && !activeController_->checkStability()) {
            switchToFallbackController();
        }

        // Compute current time and time since last call
        scalar_t currentTime = getCurrentTime();
        scalar_t dt = currentTime - lastTime;

        // Step controller
        step(currentTime, dt);

        // Allow controller to visualize stuff
        visualize();

        lastTime = currentTime;

        auto t2 = std::chrono::high_resolution_clock::now();
        loopRate_.sleep();
        auto t3 = std::chrono::high_resolution_clock::now();

        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        auto sleepTimePercentage = 100.0 * duration2 / (duration1 + duration2);

        ROS_INFO_STREAM_THROTTLE(
            5.0, "[CentralController] Loop duration: " << duration1 << " us, Sleep duration: " << duration2 << " us, "
                                                       << "Sleep time percentage: " << sleepTimePercentage << "%");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool RosCentralController::checkForFallbackController() {
    for (const auto &controller : controllers_) {
        if (controller->isSupported(fallbackControllerType_)) {
            return true;
        }
    }
    return false;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosCentralController::switchToFallbackController() {
    std_msgs::String msg;
    msg.data = fallbackControllerType_;
    changeControllerCallback(std_msgs::String::ConstPtr(new std_msgs::String(msg)));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosCentralController::step(scalar_t currentTime, scalar_t dt) {
    auto commandMessage = activeController_->getMotorCommands(currentTime, dt);
        commandPublisher_->publish(commandMessage);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
const std::shared_ptr<StateSubscriber> &RosCentralController::getStateSubscriberPtr() {
    return stateSubscriberPtr_;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosCentralController::changeControllerCallback(const std_msgs::String::ConstPtr &msg) {
    const std::string controllerType = msg->data;
    for (auto &controller : controllers_) {
        if (controller->isSupported(controllerType)) {
            if (activeController_ != controller.get()) {
                activeController_->stopController();  // Stop current controller
            }
            activeController_ = controller.get();  // Set new active controller
            activeController_->changeController(controllerType, getCurrentTime());
            loopRate_ = ros::Rate(activeController_->getRate());
            ROS_INFO_STREAM("[CentralController] Controller changed to " << controllerType);
            return;
        }
    }
    ROS_WARN_STREAM("[CentralController] Controller " << controllerType << " not supported");
}

}  // namespace tbai
