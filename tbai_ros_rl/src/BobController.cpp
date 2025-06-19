#include "tbai_ros_rl/BobController.hpp"

namespace tbai {
namespace rl {

RosBobController::RosBobController(const std::string &urdfString, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::BobController(urdfString, stateSubscriberPtr, refVelGenPtr) {
    if (!blind_) {
        gridmap_ = tbai::gridmap::getGridmapInterfaceUnique();
    }
}

void RosBobController::visualize() {
    auto state = getBobnetState();
    stateVisualizer_.visualize(state);
    heightsReconstructedVisualizer_.visualize(state, sampled_, hidden_);
}

void RosBobController::changeController(const std::string &controllerType, scalar_t currentTime) {
    if (!blind_) {
        gridmap_->waitTillInitialized();
    }
}

void RosBobController::atPositions(matrix_t &positions) {
    gridmap_->atPositions(positions);
}

void RosBobController::stopController() {
    // Implementation for stopController
}

bool RosBobController::ok() const {
    return ros::ok();
}

void RosBobController::triggerCallbacks() {
    ros::spinOnce();
}

}  // namespace rl
}  // namespace tbai 