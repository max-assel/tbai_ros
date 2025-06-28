#include <ocs2_anymal_mpc/AnymalInterface.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>
#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nodeHandle;

    // Anymal urdf
    std::string urdfString;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/robot_description", urdfString),
                      "Failed to get parameter /robot_description");

    // Task settings
    std::string taskSettingsFile;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/task_settings_file", taskSettingsFile),
                      "Failed to get parameter /task_settings_file");

    // Frame declarations
    std::string frameDeclarationFile;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get parameter /frame_declaration_file");

    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterface;
    if (robotName == "anymal_d") {
        quadrupedInterface =
            anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                       anymal::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName == "go2") {
        quadrupedInterface =
            anymal::getGo2Interface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                    anymal::frameDeclarationFromFile(frameDeclarationFile));
    } else {
        TBAI_THROW("Robot name not supported: {}", robotName);
    }

    // Prepare robot interface
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

    if (quadrupedInterface->modelSettings().algorithm_ == switched_model::Algorithm::SQP) {
        ROS_INFO_STREAM("[MpcNode] Using SQP MPC");
        std::string sqpSettingsFile;
        TBAI_THROW_UNLESS(nodeHandle.getParam("/sqp_settings_file", sqpSettingsFile),
                          "Failed to get parameter /sqp_settings_file");
        const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
        auto mpcPtr = switched_model::getSqpMpc(*quadrupedInterface, mpcSettings, sqpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    if (quadrupedInterface->modelSettings().algorithm_ == switched_model::Algorithm::DDP) {
        ROS_INFO_STREAM("[MpcNode] Using DDP MPC");
        const auto ddpSettings = ocs2::ddp::loadSettings(taskSettingsFile);
        auto mpcPtr = switched_model::getDdpMpc(*quadrupedInterface, mpcSettings, ddpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    return EXIT_SUCCESS;
}
