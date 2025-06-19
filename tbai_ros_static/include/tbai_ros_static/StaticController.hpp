#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tbai_ros_core/Subscribers.hpp"
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/control/Controllers.hpp>
#include <tf/transform_broadcaster.h>

namespace tbai {
namespace static_ {

class StaticController : public tbai::Controller {
   public:
    /**
     * @brief Construct a new StaticController object
     *
     */
    StaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr);

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override;

    bool checkStability() const override { return true; }

    bool ok() const override { return ros::ok(); }

    void triggerCallbacks() override { ros::spinOnce(); }

   private:
    void loadSettings();

    /** Publish odom->base transforms */
    void publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime);

    /** Publish joint angles */
    void publishJointAngles(const vector_t &currentState, const ros::Time &currentTime);

    /** Get command message during interpolation phase */
    std::vector<MotorCommand> getInterpCommandMessage(scalar_t dt);

    /** Get command message when standing */
    std::vector<MotorCommand> getStandCommandMessage();

    /** Get command message when sitting */
    std::vector<MotorCommand> getSitCommandMessage();

    /** Pack desired joint angles into a command message */
    std::vector<MotorCommand> packCommandMessage(const vector_t &jointAngles);

    /** State subscriber */
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    /** Visualization */
    tf::TransformBroadcaster tfBroadcaster_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    /** Time since last visualization step */
    scalar_t timeSinceLastVisualizationUpdate_;

    /** PD constants */
    scalar_t kp_;
    scalar_t kd_;

    /** Stand joint angles */
    vector_t standJointAngles_;

    /** Sit joint angles */
    vector_t sitJointAngles_;

    /** How long should interpolation take */
    scalar_t interpolationTime_;

    /** Interp from and to */
    vector_t interpFrom_;
    vector_t interpTo_;

    /** Interpolation phase: -1 means interpolation has finished */
    scalar_t alpha_;

    /** Rate at which the controller should be running */
    scalar_t rate_;

    /** Joint names */
    std::vector<std::string> jointNames_;

    /** Current controller type */
    std::string currentControllerType_;
};

}  // namespace static_
}  // namespace tbai
