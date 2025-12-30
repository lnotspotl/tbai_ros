#include "tbai_ros_ocs2/logic/GaitCommandNode.h"
#include <ros/ros.h>
#include <tbai_core/Throws.hpp>

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "go2_mpc_mode_sequence");
    ros::NodeHandle nodeHandle;

    // Get gait file from parameter server
    std::string gaitFile;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/gait_file", gaitFile), "Failed to get parameter /gait_file");

    std::string robotName;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/robot_name", robotName), "Failed to get parameter /robot_name");

    switched_model::GaitCommandNode gaitCommandNode(nodeHandle, gaitFile, robotName, true);
    while (ros::ok() && ros::master::check()) {
        gaitCommandNode.getKeyboardCommand();
    }

    return 0;
}
