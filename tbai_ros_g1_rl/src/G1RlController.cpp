#include "tbai_ros_g1_rl/G1RlController.hpp"

#include <tbai_core/config/Config.hpp>
#include <tbai_ros_msgs/EstimatedState.h>

namespace tbai {
namespace g1_rl {

RosG1RlController::RosG1RlController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::G1RlController(stateSubscriberPtr, refVelGenPtr) {
}

void RosG1RlController::postStep(scalar_t currentTime, scalar_t dt) {
    std::cout << state_.x.transpose() << std::endl;
}

void RosG1RlController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);
}

void RosG1RlController::stopController() {
    // Implementation for stopController
}

bool RosG1RlController::ok() const {
    return ros::ok();
}

void RosG1RlController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = stateSubscriberPtr_->getLatestState();
}

}  // namespace g1_rl
}  // namespace tbai