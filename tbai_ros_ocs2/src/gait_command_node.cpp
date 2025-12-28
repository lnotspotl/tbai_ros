#include <ros/package.h>
#include <ros/ros.h>

#include "tbai_ros_ocs2/logic/GaitCommandNode.h"

int main(int argc, char* argv[]) {
  std::string robotName = "anymal";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mode_sequence");
  ros::NodeHandle nodeHandle;

  // Get gait file from parameter server
  std::string gaitFile;
  if (!nodeHandle.getParam("gait_file", gaitFile)) {
    gaitFile = ros::package::getPath("tbai_ros_ocs2") + "/config/gait.info";
    ROS_WARN_STREAM("gait_file parameter not set, using default: " << gaitFile);
  }
  gaitFile = "/home/kb/Documents/ros/src/tbai_ros/tbai_ros_deploy_go2/tbai_ros_deploy_go2_mpc/config/gait.info";

  // Get robot name from parameter server
  // nodeHandle.getParam("robot_name", robotName);

  ROS_INFO_STREAM("Loading gait file: " << gaitFile);
  ROS_INFO_STREAM("Robot name: " << robotName);

  switched_model::GaitCommandNode gaitCommandNode(nodeHandle, gaitFile, robotName, true);

  while (ros::ok() && ros::master::check()) {
    gaitCommandNode.getKeyboardCommand();
  }

  return 0;
}
