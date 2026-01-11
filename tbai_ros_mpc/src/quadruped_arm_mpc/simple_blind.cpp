// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>

#include "tbai_ros_mpc/quadruped_arm_mpc/RosMpcController.hpp"
#include "tbai_ros_static/StaticController.hpp"
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_mpc_arm");
    ros::NodeHandle nh;

    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    auto stateTopic = tbai::fromGlobalConfig<std::string>("state_topic");
    auto commandTopic = tbai::fromGlobalConfig<std::string>("command_topic");
    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
    const std::string urdfString = nh.param<std::string>("robot_description", "");

    auto estimatorType = tbai::fromGlobalConfig<std::string>("estimator_type");

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber;
    if (estimatorType == "muse") {
        stateSubscriber =
            std::shared_ptr<tbai::StateSubscriber>(new tbai::InekfRosStateSubscriber(nh, stateTopic, urdfString));
    } else if (estimatorType == "ground_truth") {
        stateSubscriber = std::shared_ptr<tbai::StateSubscriber>(new tbai::RosStateSubscriber(nh, stateTopic));
    } else {
        TBAI_GLOBAL_LOG_FATAL("Invalid estimator type: {}. Must be one of 'muse' or 'ground_truth'", estimatorType);
        return EXIT_FAILURE;
    }

    std::shared_ptr<tbai::CommandPublisher> commandPublisher =
        std::shared_ptr<tbai::CommandPublisher>(new tbai::RosCommandPublisher(nh, commandTopic));

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    // Add MPC controller (arm version with 18 joints)
    controller.addController(std::make_unique<tbai::mpc::RosMpcController>(
        robotName, stateSubscriber, tbai::reference::getReferenceVelocityGeneratorShared(nh), tbai::RosTime::rightNow));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
