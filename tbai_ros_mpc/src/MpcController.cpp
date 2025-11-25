
// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_mpc/MpcController.hpp"

#include <string>
#include <vector>

#include "tbai_ros_mpc/wbc/Factory.hpp"
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {
namespace mpc {

MpcController::MpcController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr)
    : stateSubscriberPtr_(stateSubscriberPtr), mrt_("anymal"), stopReferenceThread_(false) {
    initTime_ = tbai::readInitTime();

    logger_ = tbai::getLogger("mpc_controller");

    const std::string robotName = "anymal";
    ros::NodeHandle nh;

    // Load default joint state
    std::string targetCommandConfig;
    TBAI_THROW_UNLESS(nh.getParam("/target_command_config_file", targetCommandConfig),
                      "Failed to get parameter /target_command_config_file");

    // URDF
    std::string urdfString;
    TBAI_THROW_UNLESS(nh.getParam("/robot_description", urdfString), "Failed to get parameter /robot_description");

    // Task settings
    std::string taskSettingsFile;
    TBAI_THROW_UNLESS(nh.getParam("/task_settings_file", taskSettingsFile),
                      "Failed to get parameter /task_settings_file");

    // Frame declarations
    std::string frameDeclarationFile;
    TBAI_THROW_UNLESS(nh.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get parameter /frame_declaration_file");

    // Controller config
    std::string controllerConfigFile;
    TBAI_THROW_UNLESS(nh.getParam("/controller_config_file", controllerConfigFile),
                      "Failed to get parameter /controller_config_file");

    quadrupedInterfacePtr_ =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));

    visualizerPtr_ = std::make_unique<switched_model::QuadrupedVisualizer>(quadrupedInterfacePtr_->getKinematicModel(),
                                                                           quadrupedInterfacePtr_->getJointNames(),
                                                                           quadrupedInterfacePtr_->getBaseName(), nh);
    contactVisualizerPtr_ = std::make_unique<ContactVisualizer>();

    wbcPtr_ = switched_model::getWbcUnique(controllerConfigFile, urdfString, quadrupedInterfacePtr_->getComModel(),
                                           quadrupedInterfacePtr_->getKinematicModel(),
                                           quadrupedInterfacePtr_->getJointNames());

    referenceThreadNodeHandle_.setCallbackQueue(&referenceThreadCallbackQueue_);
    referenceTrajectoryGeneratorPtr_ =
        reference::getReferenceTrajectoryGeneratorUnique(referenceThreadNodeHandle_, velocityGeneratorPtr);

    mrt_.launchNodes(nh);
    tNow_ = 0.0;
}

MpcController::~MpcController()
{
    std::chrono::steady_clock::time_point destructStartTime = std::chrono::steady_clock::now();
    std::ofstream logFile;
    logFile.open("/home/masselmeier3/Desktop/Research/quad_pips_experiments/timing/mpc/timing_log_" + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(destructStartTime.time_since_epoch()).count()) + ".csv", std::ios::out);

    float averageUpdateTime = updateTimeTaken * 1.0e-3 / static_cast<float>(numberOfUpdateCalls);
    float averageEvaluateTime = evaluateTimeTaken * 1.0e-3 / static_cast<float>(numberOfEvaluateCalls);
    float averageWbcTime = wbcTimeTaken * 1.0e-3 / static_cast<float>(numberOfWbcCalls);
    float averageTotalTime = totalTimeTaken * 1.0e-3 / static_cast<float>(numberOfTotalCalls);

    logFile << "avg update time (ms), avg evaluate time (ms), avg wbc time (ms), avg total time (ms), number of calls" << std::endl;
    logFile << averageUpdateTime << ", " << averageEvaluateTime << ", " << averageWbcTime << ", " << averageTotalTime << ", " << numberOfTotalCalls << std::endl;
    logFile.close();
}

void MpcController::spinOnceReferenceThread() {
    referenceThreadCallbackQueue_.callAvailable(ros::WallDuration(0.0));
}

std::vector<MotorCommand> MpcController::getMotorCommands(scalar_t currentTime, scalar_t dt) 
{
    // ROS_INFO_STREAM("[MpcController::getMotorCommands] start");
    totalStartTime_ = std::chrono::steady_clock::now();

    mrt_.spinMRT();

    updateStartTime_ = std::chrono::steady_clock::now();
    mrt_.updatePolicy();
    updateEndTime_ = std::chrono::steady_clock::now();
    updateTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(updateEndTime_ - updateStartTime_).count();
    numberOfUpdateCalls++;

    tNow_ = ros::Time::now().toSec() - initTime_;

    auto observation = generateSystemObservation();

    // ROS_INFO_STREAM("[MpcController::getMotorCommands] observation: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      time: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          " << observation.time);
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      state: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso orientation: " << observation.state.segment(0,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso position:    " << observation.state.segment(3,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso angular vel: " << observation.state.segment(6,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso linear vel:  " << observation.state.segment(9,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF joint pos:      " << observation.state.segment(12,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF joint pos:      " << observation.state.segment(15,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH joint pos:      " << observation.state.segment(18,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH joint pos:      " << observation.state.segment(21,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      input: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF contact force:  " << observation.input.segment(0,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF contact force:  " << observation.input.segment(3,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH contact force:  " << observation.input.segment(6,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH contact force:  " << observation.input.segment(9,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF joint vel:      " << observation.input.segment(12,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF joint vel:      " << observation.input.segment(15,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH joint vel:      " << observation.input.segment(18,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH joint vel:      " << observation.input.segment(21,3).transpose());

    ocs2::vector_t desiredState;
    ocs2::vector_t desiredInput;
    size_t desiredMode;

    evaluateStartTime_ = std::chrono::steady_clock::now();
    mrt_.evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);
    evaluateEndTime_ = std::chrono::steady_clock::now();
    evaluateTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(evaluateEndTime_ - evaluateStartTime_).count();
    numberOfEvaluateCalls++;

    // ROS_INFO_STREAM("[MpcController::getMotorCommands] desired: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      state: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso orientation: " << desiredState.segment(0,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso position:    " << desiredState.segment(3,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso angular vel: " << desiredState.segment(6,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          torso linear vel:  " << desiredState.segment(9,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF joint pos:      " << desiredState.segment(12,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF joint pos:      " << desiredState.segment(15,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH joint pos:      " << desiredState.segment(18,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH joint pos:      " << desiredState.segment(21,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      input: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF contact force:  " << desiredInput.segment(0,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF contact force:  " << desiredInput.segment(3,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH contact force:  " << desiredInput.segment(6,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH contact force:  " << desiredInput.segment(9,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF joint vel:      " << desiredInput.segment(12,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF joint vel:      " << desiredInput.segment(15,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH joint vel:      " << desiredInput.segment(18,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH joint vel:      " << desiredInput.segment(21,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      mode: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          desired mode:      " << desiredMode);

    constexpr ocs2::scalar_t time_eps = 1e-4;
    ocs2::vector_t dummyState;
    ocs2::vector_t dummyInput;
    size_t dummyMode;

    evaluateStartTime_ = std::chrono::steady_clock::now();
    mrt_.evaluatePolicy(tNow_ + time_eps, observation.state, dummyState, dummyInput, dummyMode);
    evaluateEndTime_ = std::chrono::steady_clock::now();
    evaluateTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(evaluateEndTime_ - evaluateStartTime_).count();
    numberOfEvaluateCalls++;

    // ROS_INFO_STREAM("[MpcController::getMotorCommands] dummy: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      input: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF contact force:  " << dummyInput.segment(0,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF contact force:  " << dummyInput.segment(3,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH contact force:  " << dummyInput.segment(6,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH contact force:  " << dummyInput.segment(9,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LF joint vel:      " << dummyInput.segment(12,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RF joint vel:      " << dummyInput.segment(15,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          LH joint vel:      " << dummyInput.segment(18,3).transpose());
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]          RH joint vel:      " << dummyInput.segment(21,3).transpose());

    ocs2::vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

    // ROS_INFO_STREAM("[MpcController::getMotorCommands] joint_accelerations: ");
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [0]: " << joint_accelerations(0));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [1]: " << joint_accelerations(1));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [2]: " << joint_accelerations(2));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [3]: " << joint_accelerations(3));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [4]: " << joint_accelerations(4));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [5]: " << joint_accelerations(5));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [6]: " << joint_accelerations(6));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [7]: " << joint_accelerations(7));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [8]: " << joint_accelerations(8));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [9]: " << joint_accelerations(9));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [10]: " << joint_accelerations(10));
    // ROS_INFO_STREAM("[MpcController::getMotorCommands]      [11]: " << joint_accelerations(11));

    wbcStartTime_ = std::chrono::steady_clock::now();
    auto commands = wbcPtr_->getMotorCommands(tNow_, observation.state, observation.input, observation.mode,
                                              desiredState, desiredInput, desiredMode, joint_accelerations, isStable_);
    wbcEndTime_ =  std::chrono::steady_clock::now();
    wbcTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(wbcEndTime_ - wbcStartTime_).count();
    numberOfWbcCalls++;

    // ROS_INFO_STREAM("[MpcController::getMotorCommands] motor commands: ");
    // for (size_t i = 0; i < commands.size(); ++i) 
    // {
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]      [" << i << "]");
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          joint_name:     " << commands[i].joint_name);
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          desired_position:     " << commands[i].desired_position);
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          desired_velocity:     " << commands[i].desired_velocity);
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          kp:   " << commands[i].kp);
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          kd:   " << commands[i].kd);
    //     ROS_INFO_STREAM("[MpcController::getMotorCommands]          torque_ff:   " << commands[i].torque_ff);
    // }

    timeSinceLastMpcUpdate_ += dt;
    timeSinceLastVisualizationUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_) 
    {
        setObservation();
    }

    totalEndTime_ = std::chrono::steady_clock::now();
    totalTimeTaken += std::chrono::duration_cast<std::chrono::microseconds>(totalEndTime_ - totalStartTime_).count();
    numberOfTotalCalls++;

    return commands;
}

void MpcController::referenceThread() {
    referenceTrajectoryGeneratorPtr_->reset();

    // Wait for initial mpc observation
    while (ros::ok() && !stopReferenceThread_) {
        spinOnceReferenceThread();
        if (referenceTrajectoryGeneratorPtr_->isInitialized()) break;
        ros::Duration(0.02).sleep();
    }

    // Start reference thread
    ros::Rate rate(5.0);
    while (ros::ok() && !stopReferenceThread_) {
        spinOnceReferenceThread();
        TBAI_LOG_INFO_THROTTLE(logger_, 5.0, "Publishing reference");
        referenceTrajectoryGeneratorPtr_->publishReferenceTrajectory();
        rate.sleep();
    }
}

bool MpcController::checkStability() const {
    if (!isStable_) {
        TBAI_LOG_ERROR(logger_, "Mpc is unstable");
        return false;
    }

    scalar_t roll = state_.x[0];
    if (roll >= 1.57 || roll <= -1.57) {
        return false;
    }
    scalar_t pitch = state_.x[1];
    if (pitch >= 1.57 || pitch <= -1.57) {
        return false;
    }
    return true;
}

void MpcController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 15.0) {
        visualizerPtr_->update(generateSystemObservation(), mrt_.getPolicy(), mrt_.getCommand());
        contactVisualizerPtr_->visualize(state_.x, state_.contactFlags);
        timeSinceLastVisualizationUpdate_ = 0.0;
    }
}

void MpcController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);

    if (!mrt_initialized_ || currentTime + 0.1 > mrt_.getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = currentTime;

    // Start reference thread
    startReferenceThread();
}

void MpcController::startReferenceThread() {
    // Start reference thread
    if (referenceThread_.joinable()) {
        referenceThread_.join();
    }
    stopReferenceThread_ = false;
    referenceThread_ = std::thread(&MpcController::referenceThread, this);
}

void MpcController::stopReferenceThread() {
    stopReferenceThread_ = true;
}

bool MpcController::isSupported(const std::string &controllerType) {
    return controllerType == "WBC";
}

void MpcController::resetMpc() {
    // Generate initial observation
    stateSubscriberPtr_->waitTillInitialized();
    auto initialObservation = generateSystemObservation();
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initialObservation.state},
                                                          {initialObservation.input});
    mrt_.resetMpcNode(initTargetTrajectories);

    while (!mrt_.initialPolicyReceived() && ros::ok()) {
        TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
        ros::spinOnce();
        mrt_.spinMRT();
        initialObservation = generateSystemObservation();
        mrt_.setCurrentObservation(initialObservation);
        ros::Duration(0.1).sleep();
    }

    TBAI_LOG_INFO(logger_, "Initial policy received.");
}

void MpcController::setObservation() {
    mrt_.setCurrentObservation(generateSystemObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

ocs2::SystemObservation MpcController::generateSystemObservation() const {
    auto state = stateSubscriberPtr_->getLatestState();
    const tbai::vector_t &rbdState = state.x;

    // Set observation time
    ocs2::SystemObservation observation;
    observation.time = state.timestamp - initTime_;

    // Set mode
    const std::vector<bool> &contactFlags = state.contactFlags;
    std::array<bool, 4> contactFlagsArray = {contactFlags[0], contactFlags[1], contactFlags[2], contactFlags[3]};
    observation.mode = switched_model::stanceLeg2ModeNumber(contactFlagsArray);

    // Set state
    observation.state = rbdState.head<3 + 3 + 3 + 3 + 12>();

    // Swap LH and RF
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 0), observation.state(3 + 3 + 3 + 3 + 3 + 3));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 1), observation.state(3 + 3 + 3 + 3 + 3 + 4));
    std::swap(observation.state(3 + 3 + 3 + 3 + 3 + 2), observation.state(3 + 3 + 3 + 3 + 3 + 5));

    // Set input
    observation.input.setZero(24);
    observation.input.tail<12>() = rbdState.tail<12>();

    // Swap LH and RF
    std::swap(observation.input(12 + 3), observation.input(12 + 6));
    std::swap(observation.input(12 + 4), observation.input(12 + 7));
    std::swap(observation.input(12 + 5), observation.input(12 + 8));

    return observation;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
ContactVisualizer::ContactVisualizer() {
    ros::NodeHandle nh;
    contactPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/contact_points", 1);

    odomFrame_ = "odom";
    footFrameNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // Setup Pinocchio model
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void ContactVisualizer::visualize(const vector_t &currentState, const std::vector<bool> &contacts) {
    ros::Time timeStamp = ros::Time::now();

    vector_t q = vector_t::Zero(model_.nq);
    q.head<3>() = currentState.segment<3>(3);                               // Position
    q.segment<4>(3) = tbai::ocs2rpy2quat(currentState.head<3>()).coeffs();  // Orientation
    q.tail(12) = currentState.segment<12>(3 + 3 + 3 + 3);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < footFrameNames_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = timeStamp;
        marker.header.frame_id = odomFrame_;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = contacts[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.pose.position.x = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[0];
        marker.pose.position.y = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[1];
        marker.pose.position.z = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        markerArray.markers.push_back(marker);
    }
    contactPublisher_.publish(markerArray);
}

}  // namespace mpc
}  // namespace tbai
