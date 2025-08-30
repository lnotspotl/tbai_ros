#include <ros/ros.h>
#include <tbai_ros_deploy_go2_rl/Go2Joystick.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "go2_joystick_node");
    ros::NodeHandle nh;

    tbai::reference::Go2Joystick joystick(nh);
    joystick.Start();

    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        joystick.publishTwist();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}