#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_g1_rl/G1RlController.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_gridmap/GridmapInterface.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

namespace tbai {
namespace g1_rl {

using namespace tbai;             // NOLINT

class RosG1RlController : public tbai::G1RlController {
   public:
    RosG1RlController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr);

    void postStep(scalar_t currentTime, scalar_t dt) override;
    void changeController(const std::string &controllerType, scalar_t currentTime) override;
    void stopController() override;
    bool ok() const override;
    void preStep(scalar_t currentTime, scalar_t dt) override;

   private:
};

}  // namespace g1_rl
}  // namespace tbai
