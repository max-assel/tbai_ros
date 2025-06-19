#include "tbai_ros_gazebo/JointController.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <tbai_core/Logging.hpp>
#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool JointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    // Setup yaml config
    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load joint names
    TBAI_LOG_INFO("Loading {} joint controllers for joints {}", jointNames.size(), jointNames);

    // Load model URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {  // TODO(lnotspotl): Parametrize this
        TBAI_LOG_FATAL("Could not parse urdf file! Failed to initialize.");
        return false;
    }

    for (size_t i = 0; i < jointNames.size(); ++i) {
        std::string &jointName = jointNames[i];

        // Get joint handle
        try {
            jointHandles_.push_back(hw->getHandle(jointName));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            TBAI_LOG_FATAL("Could not find joint '{}' in hardware interface", jointName);
            return false;
        }

        // Get joint position limits
        std::pair<tbai::scalar_t, tbai::scalar_t> jointLimit;
        urdf::JointConstSharedPtr jointUrdf = urdf.getJoint(jointName);
        if (!jointUrdf) {
            TBAI_LOG_FATAL("Could not find joint '{}' in urdf", jointName);
            return false;
        }
        jointLimit.first = jointUrdf->limits->lower;
        jointLimit.second = jointUrdf->limits->upper;
        jointLimits_.push_back(jointLimit);

        // Get joint effort limits
        effortLimits_.push_back(jointUrdf->limits->effort);

        // Get joint index
        jointIndexMap_[jointName] = i;

        TBAI_LOG_INFO("Loaded joint '{}' with limits [{}, {}] and effort limit {}", jointName, jointLimit.first,
                      jointLimit.second, jointUrdf->limits->effort);
    }

    // Load command topic
    auto commandTopic = tbai::fromGlobalConfig<std::string>("command_topic");
    TBAI_LOG_INFO("Subscribing to {} for joint commands", commandTopic);

    // Subscribe to command topic
    ros::NodeHandle nh;
    commandSubscriber_ =
        nh.subscribe<tbai_ros_msgs::JointCommandArray>(commandTopic, 1, &JointController::jointCommandCallback, this);

    // Initialize command buffer
    commandBuffer_.writeFromNonRT(tbai_ros_msgs::JointCommandArray());

    // Store number of joints
    numJoints_ = jointNames.size();

    return true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void JointController::update(const ros::Time &time, const ros::Duration &period) {
    // Read command buffer
    tbai_ros_msgs::JointCommandArray &commandArray = *commandBuffer_.readFromRT();

    // Update last joint positions if this is the first update
    if (firstUpdate_) {
        lastJointPositions_.resize(numJoints_);
        for (size_t i = 0; i < numJoints_; ++i) {
            lastJointPositions_[i] = jointHandles_[i].getPosition();
        }
        firstUpdate_ = false;
    }

    const scalar_t dt = period.toSec();
    const size_t nCommands = commandArray.joint_commands.size();
    for (int i = 0; i < nCommands; ++i) {
        // Unpack JointCommand message
        tbai_ros_msgs::JointCommand &command = commandArray.joint_commands[i];
        std::string &jointName = command.joint_name;
        const scalar_t desiredPosition = command.desired_position;
        const scalar_t desiredVelocity = command.desired_velocity;
        const scalar_t kp = command.kp;
        const scalar_t kd = command.kd;
        const scalar_t torque_ff = command.torque_ff;

        // Get joint handle
        const size_t jointIdx = jointIndexMap_[jointName];
        hardware_interface::JointHandle &joint = jointHandles_[jointIdx];

        // Get current joint position
        const scalar_t currentPosition = joint.getPosition();

        // Get current joint velocity
        const scalar_t currentVelocity = (currentPosition - lastJointPositions_[jointIdx]) / dt;

        // Update last joint position
        lastJointPositions_[jointIdx] = currentPosition;

        // Compute torque
        const scalar_t positionError = desiredPosition - currentPosition;
        const scalar_t velocityError = desiredVelocity - currentVelocity;
        const scalar_t torque = torque_ff + kp * positionError + kd * velocityError;

        // Clip torque
        const scalar_t effortLimit = effortLimits_[jointIdx];
        const scalar_t torque_clipped = std::max(-effortLimit, std::min(effortLimit, torque));

        // Set torque
        joint.setCommand(torque_clipped);
    }
}

void JointController::jointCommandCallback(const tbai_ros_msgs::JointCommandArrayConstPtr &commandArrayPtr) {
    commandBuffer_.writeFromNonRT(*commandArrayPtr);
}

}  // namespace gazebo
}  // namespace tbai

PLUGINLIB_EXPORT_CLASS(tbai::gazebo::JointController, controller_interface::ControllerBase);
