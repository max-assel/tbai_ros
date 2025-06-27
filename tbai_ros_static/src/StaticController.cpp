// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_static/StaticController.hpp"

#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/config/Config.hpp>
#include <urdf/model.h>

namespace tbai {
namespace static_ {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosStaticController::RosStaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr)
    : StaticController(stateSubscriberPtr) {
    // Setup state publisher
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    KDL::Tree kdlTree;
    kdl_parser::treeFromString(urdfString, kdlTree);
    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    // Some dummy value
    timeSinceLastVisualizationUpdate_ = 1000.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        auto currentRosTime = ros::Time::now();
        publishOdomBaseTransforms(state_.x, currentRosTime);
        publishJointAngles(state_.x, currentRosTime);
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime) {
    geometry_msgs::TransformStamped odomBaseTransform;

    // Header
    odomBaseTransform.header.stamp = currentTime;
    odomBaseTransform.header.frame_id = "odom";
    odomBaseTransform.child_frame_id = "base";

    // Position
    odomBaseTransform.transform.translation.x = currentState(3);
    odomBaseTransform.transform.translation.y = currentState(4);
    odomBaseTransform.transform.translation.z = currentState(5);

    // Orientation
    tbai::quaternion_t quat = tbai::ocs2rpy2quat(currentState.head<3>());
    odomBaseTransform.transform.rotation.x = quat.x();
    odomBaseTransform.transform.rotation.y = quat.y();
    odomBaseTransform.transform.rotation.z = quat.z();
    odomBaseTransform.transform.rotation.w = quat.w();

    // Publish
    tfBroadcaster_.sendTransform(odomBaseTransform);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::publishJointAngles(const vector_t &currentState, const ros::Time &currentTime) {
    std::map<std::string, scalar_t> jointPositionMap;
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        jointPositionMap[jointNames_[i]] = currentState(i + 3 + 3 + 3 + 3);
    }
    robotStatePublisherPtr_->publishTransforms(jointPositionMap, currentTime);
}

}  // namespace static_
}  // namespace tbai
