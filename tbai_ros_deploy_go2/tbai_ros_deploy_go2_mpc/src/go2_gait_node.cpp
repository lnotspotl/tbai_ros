
#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace loadData = ocs2::loadData;
using ocs2::getCommandLineString;
using ocs2::stringToWords;
using ocs2::legged_robot::createModeSequenceTemplateMsg;
using ocs2::legged_robot::loadModeSequenceTemplate;
using ocs2::legged_robot::ModeSequenceTemplate;

class GaitKeyboardPublisher {
   public:
    GaitKeyboardPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile, const std::string &robotName,
                          bool verbose = false);

    std::string getKeyboardCommand();

    void handleGaitCommand(std::string gaitCommand);

    void gaitCommandCallback(const std_msgs::String::ConstPtr &msg);

    void printGaitList(const std::vector<std::string> &gaitList) const;

    std::vector<std::string> gaitList_;
    std::map<std::string, ModeSequenceTemplate> gaitMap_;

    ros::Publisher modeSequenceTemplatePublisher_;
    ros::Subscriber gaitCommandSubscriber_;
    std::string gaitCommand_;
    std::mutex gaitCommandMutex_;
};

GaitKeyboardPublisher::GaitKeyboardPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile,
                                             const std::string &robotName, bool verbose) {
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
    loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

    modeSequenceTemplatePublisher_ =
        nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);
    gaitCommandSubscriber_ =
        nodeHandle.subscribe<std_msgs::String>("/gait_command", 1, &GaitKeyboardPublisher::gaitCommandCallback, this);

    gaitMap_.clear();
    for (const auto &gaitName : gaitList_) {
        gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
    }
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void GaitKeyboardPublisher::gaitCommandCallback(const std_msgs::String::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(gaitCommandMutex_);
    gaitCommand_ = msg->data;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string GaitKeyboardPublisher::getKeyboardCommand() {
    const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
    std::cout << commadMsg << ": ";

    auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
    const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

    if (commandLine.empty()) {
        return "";
    }

    if (commandLine.size() > 1) {
        std::cout << "WARNING: The command should be a single word." << std::endl;
        return "";
    }

    std::lock_guard<std::mutex> lock(gaitCommandMutex_);
    handleGaitCommand(commandLine.front());
    return commandLine.front();
}

void GaitKeyboardPublisher::handleGaitCommand(std::string gaitCommand) {
    std::cout << "Gait command: " << gaitCommand << std::endl;
    std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

    if (gaitCommand == "list") {
        printGaitList(gaitList_);
        return;
    }

    try {
        ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
        modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
    } catch (const std::out_of_range &e) {
        std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
        printGaitList(gaitList_);
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(const std::vector<std::string> &gaitList) const {
    std::cout << "List of available gaits:\n";
    size_t itr = 0;
    for (const auto &s : gaitList) {
        std::cout << "[" << itr++ << "]: " << s << "\n";
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[]) {
    const std::string robotName = "anymal";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mpc_mode_schedule");
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string gaitCommandFile;
    nodeHandle.getParam("/gait_command_file", gaitCommandFile);
    std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

    GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);

    auto thread1 = std::thread([&]() {
        while (ros::ok() && ros::master::check()) {
            ros::spinOnce();
            std::string currentGaitCommand;
            {
                std::lock_guard<std::mutex> lock(gaitCommand.gaitCommandMutex_);
                currentGaitCommand = gaitCommand.gaitCommand_;
                if (!currentGaitCommand.empty()) {
                    gaitCommand.gaitCommand_ = "";
                }
            }
            if (!currentGaitCommand.empty()) {
                gaitCommand.handleGaitCommand(currentGaitCommand);
            }
            ros::Duration(0.2).sleep();
        }
    });

    while (ros::ok() && ros::master::check()) {
        std::string gaitCommand3 = gaitCommand.getKeyboardCommand();
        if (gaitCommand3 != "") {
            gaitCommand.handleGaitCommand(gaitCommand3);
            std::lock_guard<std::mutex> lock(gaitCommand.gaitCommandMutex_);
            gaitCommand.gaitCommand_ = "";
        }
    }

    thread1.join();

    // Successful exit
    return 0;
}
