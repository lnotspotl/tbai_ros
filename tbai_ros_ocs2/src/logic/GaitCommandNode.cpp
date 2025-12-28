#include "tbai_ros_ocs2/logic/GaitCommandNode.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include <tbai_ros_ocs2/mode_schedule.h>

namespace switched_model {

namespace {
/** Convert mode sequence template to tbai_ros_ocs2 ROS message */
tbai_ros_ocs2::mode_schedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& modeSequenceTemplate) {
  tbai_ros_ocs2::mode_schedule modeScheduleMsg;
  modeScheduleMsg.eventTimes.assign(modeSequenceTemplate.switchingTimes.begin(), modeSequenceTemplate.switchingTimes.end());
  modeScheduleMsg.modeSequence.assign(modeSequenceTemplate.modeSequence.begin(), modeSequenceTemplate.modeSequence.end());
  return modeScheduleMsg;
}
}  // namespace

GaitCommandNode::GaitCommandNode(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName,
                                 bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle.advertise<tbai_ros_ocs2::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void GaitCommandNode::getKeyboardCommand() {
  const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": ";

  auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
  const auto gaitCommand = ocs2::getCommandLineString(shouldTerminate);

  if (gaitCommand.empty()) {
    return;
  }

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    publishModeSequenceTemplate(modeSequenceTemplate);
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

void GaitCommandNode::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

void GaitCommandNode::publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule) {
  std::cerr << "Publishing mode sequence template" << std::endl;
  modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSchedule));
}

}  // namespace switched_model
