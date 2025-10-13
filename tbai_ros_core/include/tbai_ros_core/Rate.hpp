#pragma once

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <tbai_core/Types.hpp>

namespace tbai {
struct RosTime {
    static inline scalar_t rightNow() { return rclcpp::Clock().now().seconds(); }
};
}  // namespace tbai