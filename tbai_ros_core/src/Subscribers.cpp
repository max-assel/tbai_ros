#include "tbai_ros_core/Subscribers.hpp"

#include <mutex>

#include <tbai_core/Rotations.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>

namespace tbai {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosStateSubscriber::RosStateSubscriber(const rclcpp::Node::SharedPtr & node, const std::string &stateTopic) {
    node_ = node;
    stateSubscriber_ = node->create_subscription<tbai_ros_msgs::RbdState>(stateTopic, 1, std::bind(&RosStateSubscriber::stateMessageCallback, this, std::placeholders::_1));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStateSubscriber::waitTillInitialized() {
    while (!stateMessage_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        // ros::Duration(0.05).sleep();
        rclcpp::Rate(20).sleep();
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[StateSubscriber] Waiting for state message...");
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

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
MuseRosStateSubscriber::MuseRosStateSubscriber(const rclcpp::Node::SharedPtr & nodetemp, const std::string &stateTopic,
                                               const std::string &urdf) {
    logger_ = tbai::getLogger("MuseRosStateSubscriber");

    // ros::NodeHandle nh;
    // nh.setCallbackQueue(&thisQueue_);
    callbackGroup_ = nodetemp->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callbackGroup_;

    TBAI_LOG_INFO(logger_, "Initializing MuseRosStateSubscriber");
    stateSubscriber_ = nodetemp->create_subscription<tbai_ros_msgs::RobotState>(
        stateTopic, 1, std::bind(&MuseRosStateSubscriber::stateMessageCallback, this, std::placeholders::_1), options);

    TBAI_LOG_INFO(logger_, "Initialized MuseRosStateSubscriber");

    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    estimator_ = std::make_unique<tbai::muse::MuseEstimator>(footNames, urdf);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::waitTillInitialized() {
    if (!isRunning_) {
        startThreads();
    }

    TBAI_THROW_UNLESS(isRunning_, "MuseRosStateSubscriber not running");
    while (!stateMessage_ && rclcpp::ok()) {
        rclcpp::spin_some(node_);
        // ros::Duration(0.05).sleep();
        rclcpp::Rate(20).sleep();
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[StateSubscriber] Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::startThreads() {
    isRunning_ = true;
    stateThread_ = std::thread(&MuseRosStateSubscriber::threadFunction, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::stopThreads() {
    isRunning_ = false;
    if (stateThread_.joinable()) {
        stateThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::threadFunction() {
    while (isRunning_ && rclcpp::ok()) {
        thisQueue_.callAvailable(ros::WallDuration(1.0 / 5.0));
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void MuseRosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg) {
    // Determine time step since last state
    scalar_t dt = 0.0;
    scalar_t currentTime = msg->stamp.toSec();
    if (firstState_) {
        lastStateTime_ = msg->stamp;
        firstState_ = false;
    } else {
        dt = currentTime - lastStateTime_.toSec();
        lastStateTime_ = msg->stamp;
    }

    // Unpack base orientation
    vector4_t baseOrientation;
    baseOrientation[0] = msg->orientation_xyzw[0];
    baseOrientation[1] = msg->orientation_xyzw[1];
    baseOrientation[2] = msg->orientation_xyzw[2];
    baseOrientation[3] = msg->orientation_xyzw[3];

    const quaternion_t baseQuaternion(baseOrientation[3], baseOrientation[0], baseOrientation[1], baseOrientation[2]);
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    std::vector<bool> contactFlags = {msg->contact_flags[0], msg->contact_flags[1], msg->contact_flags[2],
                                      msg->contact_flags[3]};

    State state;
    state.x = vector_t::Zero(36);

    vector_t jointAngles = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointAngles[i] = msg->joint_angles[i];
    }

    vector_t jointVelocities = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointVelocities[i] = msg->joint_velocities[i];
    }

    vector3_t baseAcc = vector3_t::Zero();
    baseAcc[0] = msg->lin_acc[0];
    baseAcc[1] = msg->lin_acc[1];
    baseAcc[2] = msg->lin_acc[2];

    vector3_t baseAngVel = vector3_t::Zero();
    baseAngVel[0] = msg->ang_vel[0];
    baseAngVel[1] = msg->ang_vel[1];
    baseAngVel[2] = msg->ang_vel[2];

    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    state.x.segment<3>(0) = rpy;

    // Base position
    state.x.segment<3>(3) = estimator_->getBasePosition();

    // Base angular velocity
    state.x.segment<3>(6) = baseAngVel;

    // Base linear velocity
    state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();

    // Joint positions
    state.x.segment<12>(12) = jointAngles;

    // Joint velocities
    state.x.segment<12>(12 + 12) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    isInitialized_ = true;
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_ = std::move(state);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
InekfRosStateSubscriber::InekfRosStateSubscriber(rconst rclcpp::Node::SharedPtr & nodetemp, const std::string &stateTopic,
                                                 const std::string &urdf) {
    logger_ = tbai::getLogger("inekf_ros_state_subscriber");
    TBAI_LOG_INFO(logger_, "Initializing InekfRosStateSubscriber");

    ros::NodeHandle nh;
    nh.setCallbackQueue(&thisQueue_);
    stateSubscriber_ = nh.subscribe(stateTopic, 1, &InekfRosStateSubscriber::stateMessageCallback, this);

    std::vector<std::string> footNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    TBAI_LOG_INFO(logger_, "Creating InEKFEstimator, foot frames: {}", footNames);
    estimator_ = std::make_unique<tbai::inekf::InEKFEstimator>(footNames, urdf);

    rectifyOrientation_ = tbai::fromGlobalConfig<bool>("inekf_estimator/rectify_orientation", true);
    removeGyroscopeBias_ = tbai::fromGlobalConfig<bool>("inekf_estimator/remove_gyroscope_bias", true);
    TBAI_LOG_INFO(logger_, "Rectify orientation: {}", rectifyOrientation_);
    TBAI_LOG_INFO(logger_, "Remove gyroscope bias: {}", removeGyroscopeBias_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::waitTillInitialized() {
    if (!isRunning_) {
        startThreads();
    }

    TBAI_THROW_UNLESS(isRunning_, "MuseRosStateSubscriber not running");
    while (!isInitialized_ && rclcpp::ok()) {
        ros::spinOnce();
        ros::Duration(1.0 / 5.0).sleep();
        TBAI_LOG_INFO(logger_, "Waiting for state message...");
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::startThreads() {
    isRunning_ = true;
    stateThread_ = std::thread(&InekfRosStateSubscriber::threadFunction, this);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::stopThreads() {
    isRunning_ = false;
    if (stateThread_.joinable()) {
        stateThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::threadFunction() {
    while (isRunning_ && rclcpp::ok()) {
        thisQueue_.callAvailable(ros::WallDuration(1.0 / 5.0));
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void InekfRosStateSubscriber::stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg) {
    // Determine time step since last state
    scalar_t dt = 0.0;
    scalar_t currentTime = msg->stamp.toSec();
    if (firstState_) {
        lastStateTime_ = msg->stamp;
        firstState_ = false;
    } else {
        dt = currentTime - lastStateTime_.toSec();
        lastStateTime_ = msg->stamp;
    }

    // Unpack base orientation
    vector4_t baseOrientation;
    baseOrientation[0] = msg->orientation_xyzw[0];
    baseOrientation[1] = msg->orientation_xyzw[1];
    baseOrientation[2] = msg->orientation_xyzw[2];
    baseOrientation[3] = msg->orientation_xyzw[3];

    std::vector<bool> contactFlags = {msg->contact_flags[0], msg->contact_flags[1], msg->contact_flags[2],
                                      msg->contact_flags[3]};

    State state;
    state.x = vector_t::Zero(36);

    vector_t jointAngles = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointAngles[i] = msg->joint_angles[i];
    }

    vector_t jointVelocities = vector_t::Zero(12);
    for (int i = 0; i < 12; i++) {
        jointVelocities[i] = msg->joint_velocities[i];
    }

    vector3_t baseAcc = vector3_t::Zero();
    baseAcc[0] = msg->lin_acc[0];
    baseAcc[1] = msg->lin_acc[1];
    baseAcc[2] = msg->lin_acc[2];

    vector3_t baseAngVel = vector3_t::Zero();
    baseAngVel[0] = msg->ang_vel[0];
    baseAngVel[1] = msg->ang_vel[1];
    baseAngVel[2] = msg->ang_vel[2];

    estimator_->update(currentTime, dt, baseOrientation, jointAngles, jointVelocities, baseAcc, baseAngVel,
                       contactFlags, rectifyOrientation_, enable_);

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    const quaternion_t baseQuaternion =
        rectifyOrientation_ ? quaternion_t(baseOrientation) : quaternion_t(estimator_->getBaseOrientation());
    const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
    const tbai::matrix3_t R_base_world = R_world_base.transpose();
    const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
    lastYaw_ = rpy[2];

    // Base orientation - Euler zyx as {roll, pitch, yaw}
    state.x.segment<3>(0) = rpy;

    // Base position
    state.x.segment<3>(3) = estimator_->getBasePosition();

    // Base angular velocity
    if (removeGyroscopeBias_) {
        state.x.segment<3>(6) = baseAngVel - estimator_->getGyroscopeBias();
    } else {
        state.x.segment<3>(6) = baseAngVel;
    }

    // Base linear velocity
    state.x.segment<3>(9) = R_base_world * estimator_->getBaseVelocity();

    // Joint positions
    state.x.segment<12>(12) = jointAngles;

    // Joint velocities
    state.x.segment<12>(12 + 12) = jointVelocities;

    state.timestamp = currentTime;
    state.contactFlags = contactFlags;

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        state_ = std::move(state);
    }

    isInitialized_ = true;
}

}  // namespace tbai
