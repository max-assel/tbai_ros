#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <tbai_core/control/StateSubscriber.hpp>

#include "tbai_ros_core/Types.hpp"
#include "tbai_ros_msgs/RbdState.h"
#include <ros/ros.h>

namespace tbai {
class RosStateSubscriber : public tbai::StateSubscriber {
   public:
    RosStateSubscriber(ros::NodeHandle &nh, const std::string &stateTopic);

    void waitTillInitialized() override;

    const vector_t &getLatestRbdState() override;

    const scalar_t getLatestRbdStamp() override { return stateMessage_->stamp.toSec(); }

    const std::vector<bool> getContactFlags() override {
        return std::vector<bool>(stateMessage_->contact_flags.begin(), stateMessage_->contact_flags.end());
    }

   private:
    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::RbdState::Ptr &msg);

    /** Convert state message to vector_t */
    void updateLatestRbdState();

    /** Shared pointer to the latest state message */
    tbai_ros_msgs::RbdState::Ptr stateMessage_;

    /** State message subscriber */
    ros::Subscriber stateSubscriber_;

    /** Whether or not the latest message has been converted to vector_t */
    bool stateReady_ = false;

    /** Latest Rbd state */
    vector_t latestRbdState_;
};

}  // namespace tbai
