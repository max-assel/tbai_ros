#include "tbai_ros_rl/BobController.hpp"

namespace tbai {
namespace rl {

RosBobController::RosBobController(const std::string &urdfString,
                                   const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::BobController(urdfString, stateSubscriberPtr, refVelGenPtr) {
    if (!blind_) {
        gridmap_ = tbai::gridmap::getGridmapInterfaceUnique();
    }
    timeSinceLastVisualizationUpdate_ = 1000.0;
}

void RosBobController::visualize(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        auto state = getBobnetState();
        stateVisualizer_.visualize(state);
        heightsReconstructedVisualizer_.visualize(state, sampled_, hidden_);
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
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