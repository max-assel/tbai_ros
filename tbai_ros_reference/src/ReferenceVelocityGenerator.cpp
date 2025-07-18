#include "tbai_ros_reference/ReferenceVelocityGenerator.hpp"

#include <tbai_core/config/Config.hpp>
namespace tbai {
namespace reference {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline scalar_t sign(scalar_t x) {
    return x >= static_cast<scalar_t>(0) ? static_cast<scalar_t>(1) : static_cast<scalar_t>(-1);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void JoystickReferenceVelocityGenerator::callback(const sensor_msgs::Joy &msg) {
    referenceDesired_ = {msg.axes[xIndex_] * xScale_, msg.axes[yIndex_] * yScale_, msg.axes[yawIndex_] * yawScale_};
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
ReferenceVelocity JoystickReferenceVelocityGenerator::getReferenceVelocity(scalar_t time, scalar_t dt) {
    scalar_t x_diff = referenceDesired_.velocity_x - reference_.velocity_x;
    scalar_t y_diff = referenceDesired_.velocity_y - reference_.velocity_y;
    scalar_t yaw_diff = referenceDesired_.yaw_rate - reference_.yaw_rate;

    scalar_t x_step = sign(x_diff) * std::min(std::abs(x_diff), rampedVelocity_ * dt);
    scalar_t y_step = sign(y_diff) * std::min(std::abs(y_diff), rampedVelocity_ * dt);
    scalar_t yaw_step = sign(yaw_diff) * std::min(std::abs(yaw_diff), rampedVelocity_ * dt);

    reference_.velocity_x += x_step;
    reference_.velocity_y += y_step;
    reference_.yaw_rate += yaw_step;

    return reference_;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void TwistReferenceVelocityGenerator::callback(const geometry_msgs::Twist &msg) {
    reference_ = {msg.linear.x, msg.linear.y, msg.angular.z};
}

std::unique_ptr<ReferenceVelocityGenerator> getReferenceVelocityGeneratorUnique(ros::NodeHandle &nh) {
    auto type = tbai::fromGlobalConfig<std::string>("reference_generator/type");

    if (type == "joystick") {
        auto topic = tbai::fromGlobalConfig<std::string>("reference_generator/joystick/topic");
        auto rampedVelocity = tbai::fromGlobalConfig<scalar_t>("reference_generator/joystick/ramped_velocity");
        auto xIndex = tbai::fromGlobalConfig<size_t>("reference_generator/joystick/x_index");
        auto yIndex = tbai::fromGlobalConfig<size_t>("reference_generator/joystick/y_index");
        auto yawIndex = tbai::fromGlobalConfig<size_t>("reference_generator/joystick/yaw_index");
        auto xScale = tbai::fromGlobalConfig<scalar_t>("reference_generator/joystick/x_scale");
        auto yScale = tbai::fromGlobalConfig<scalar_t>("reference_generator/joystick/y_scale");
        auto yawScale = tbai::fromGlobalConfig<scalar_t>("reference_generator/joystick/yaw_scale");

        return std::make_unique<JoystickReferenceVelocityGenerator>(nh, topic, rampedVelocity, xIndex, yIndex, yawIndex,
                                                                    xScale, yScale, yawScale);
    }

    if (type == "twist") {
        auto topic = tbai::fromGlobalConfig<std::string>("reference_generator/twist/topic");
        return std::make_unique<TwistReferenceVelocityGenerator>(nh, topic);
    }

    throw std::runtime_error("Unknown reference generator type");
}

std::shared_ptr<ReferenceVelocityGenerator> getReferenceVelocityGeneratorShared(ros::NodeHandle &nh) {
    return std::shared_ptr<ReferenceVelocityGenerator>(getReferenceVelocityGeneratorUnique(nh).release());
}

}  // namespace reference
}  // namespace tbai
