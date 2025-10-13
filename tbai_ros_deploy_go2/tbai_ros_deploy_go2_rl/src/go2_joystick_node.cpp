#include <ros/ros.h>
#include <tbai_ros_deploy_go2_rl/Go2Joystick.hpp>

#include <tbai_core/Env.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "go2_joystick_node");
    ros::NodeHandle nh;

    tbai::reference::Go2Joystick joystick(nh);
    joystick.InitDdsModel(tbai::getEnvAs<std::string>("TBAI_GO2_NETWORK_INTERFACE", true, "eth0"));
    joystick.Start();

    ros::Rate rate(20);

    while (rclcpp::ok()) {
        ros::spinOnce();
        joystick.publishTwist();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
