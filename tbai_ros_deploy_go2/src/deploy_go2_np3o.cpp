#include <iostream>
#include <memory>

#include <tbai_ros_np3o/Np3oController.hpp>
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#include <tbai_ros_deploy_go2/Go2Joystick.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_np3o");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize Go2RobotInterface
    std::shared_ptr<tbai::Go2RobotInterface> go2RobotInterface = std::shared_ptr<tbai::Go2RobotInterface>(
        new tbai::Go2RobotInterface(tbai::Go2RobotInterfaceArgs().networkInterface("enp3s0")));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = go2RobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = go2RobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    std::string urdfString = nh.param<std::string>("robot_description", "");

    if (urdfString.empty()) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    auto joystick = tbai::reference::getGo2JoystickShared(nh);
    joystick->Start();
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> joystick_ptr = joystick;

    // Add NP3O controller
    controller.addController(std::make_unique<tbai::np3o::RosNp3oController>(
        urdfString, stateSubscriber, joystick_ptr));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
