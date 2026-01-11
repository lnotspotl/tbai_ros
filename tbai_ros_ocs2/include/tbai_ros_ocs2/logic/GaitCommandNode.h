#pragma once

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <tbai_mpc/quadruped_mpc/logic/ModeSequenceTemplate.h>

namespace tbai::mpc::quadruped {

/** This class implements gait command publishing using ROS with tbai_ros_ocs2 messages. */
class GaitCommandNode {
 public:
  GaitCommandNode(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName, bool verbose = false);

  /** Prints the command line interface and responds to user input. Function returns after one user input. */
  void getKeyboardCommand();

 private:
  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  /** Publishes the mode sequence template. */
  void publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule);

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  ros::Publisher modeSequenceTemplatePublisher_;
};

}  // namespace tbai::mpc::quadruped
