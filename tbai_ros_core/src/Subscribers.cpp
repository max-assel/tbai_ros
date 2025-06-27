#include "tbai_ros_core/Subscribers.hpp"

namespace tbai {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosStateSubscriber::RosStateSubscriber(ros::NodeHandle &nh, const std::string &stateTopic) {
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &RosStateSubscriber::stateMessageCallback, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStateSubscriber::waitTillInitialized() {
    while (!stateMessage_ && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
        ROS_INFO_STREAM_THROTTLE(1, "[StateSubscriber] Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
State RosStateSubscriber::getLatestState() {
    State state;
    state.x = vector_t(Eigen::Map<vector_t>(stateMessage_->rbd_state.data(), stateMessage_->rbd_state.size()));
    state.timestamp = stateMessage_->stamp.toSec();
    state.contactFlags = std::vector<bool>(stateMessage_->contact_flags.begin(), stateMessage_->contact_flags.end());
    return state;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RbdState::Ptr &msg) {
    stateMessage_ = msg;
}

}  // namespace tbai
