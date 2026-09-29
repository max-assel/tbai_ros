#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>

#include <ocs2_mpc/MPC_BASE.h>
#include <ros/ros.h>
#include <tbai_ros_msgs/Runtime.h>

namespace tbai {
namespace mpc {

// Time the actual MPC computation, not policy receipt/evaluation in the controller.
class TimedMpc final : public ocs2::MPC_BASE {
 public:
    TimedMpc(ros::NodeHandle& nh, std::unique_ptr<ocs2::MPC_BASE> mpc)
        : ocs2::MPC_BASE(mpc->settings()), mpc_(std::move(mpc)),
          publisher_(nh.advertise<tbai_ros_msgs::Runtime>("/benchmark/runtime/mpc", 1000)) {}

    void reset() override { mpc_->reset(); }

    bool run(ocs2::scalar_t currentTime, const ocs2::vector_t& currentState) override {
        tbai_ros_msgs::Runtime sample;
        sample.header.stamp = ros::Time::now();
        const auto start = std::chrono::steady_clock::now();
        const bool updated = mpc_->run(currentTime, currentState);
        const auto end = std::chrono::steady_clock::now();
        if (updated) {
            sample.duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
            publisher_.publish(sample);
        }
        return updated;
    }

    ocs2::SolverBase* getSolverPtr() override { return mpc_->getSolverPtr(); }
    const ocs2::SolverBase* getSolverPtr() const override { return mpc_->getSolverPtr(); }

 private:
    void calculateController(ocs2::scalar_t, const ocs2::vector_t&, ocs2::scalar_t) override {
        throw std::logic_error("TimedMpc delegates run() to the wrapped MPC");
    }

    std::unique_ptr<ocs2::MPC_BASE> mpc_;
    ros::Publisher publisher_;
};

}  // namespace mpc
}  // namespace tbai
