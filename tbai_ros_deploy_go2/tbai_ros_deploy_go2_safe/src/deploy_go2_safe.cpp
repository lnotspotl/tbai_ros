// clang-format off
#include <tbai_ros_bob/BobController.hpp>
#include <tbai_ros_np3o/Np3oController.hpp>
// clang-format on

#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/ResourceMonitor.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_deploy_go2_safe/Go2Joystick.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_go2_rl");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize Go2RobotInterface
    std::shared_ptr<tbai::Go2RobotInterface> go2RobotInterface =
        std::shared_ptr<tbai::Go2RobotInterface>(new tbai::Go2RobotInterface(
            tbai::Go2RobotInterfaceArgs()
                .networkInterface(tbai::getEnvAs<std::string>("TBAI_GO2_NETWORK_INTERFACE", true, "eth0"))
                .subscribeLidar(tbai::getEnvAs<bool>("TBAI_GO2_PUBLISH_LIDAR", true, false))
                .unitreeChannel(tbai::getEnvAs<int>("TBAI_GO2_UNITREE_CHANNEL", true, 0))));

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

    auto referenceGeneratorType = tbai::fromGlobalConfig<std::string>("reference_generator/type");

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> referenceVelocityPtr;
    if (referenceGeneratorType == "go2_joystick") {
        auto joystick = tbai::reference::getGo2JoystickShared(nh);
        joystick->Start();
        referenceVelocityPtr = joystick;
    } else if (referenceGeneratorType == "joystick") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else if (referenceGeneratorType == "twist") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else {
        TBAI_THROW("Invalid reference generator type: {}. Supported types are: go2_joystick, joystick, twist",
                   referenceGeneratorType);
    }

    // Add NP3O controller
    controller.addController(
        std::make_unique<tbai::np3o::RosNp3oController>(urdfString, stateSubscriber, referenceVelocityPtr));

    // Add Bob controller
    controller.addController(
        std::make_unique<tbai::rl::RosBobController>(urdfString, stateSubscriber, referenceVelocityPtr));

    // Start the resource monitor
    tbai::ResourceMonitor resourceMonitor(1.0 / 30.0, 1.0 / 10.0);
    resourceMonitor.startThread();

    // Start controller loop
    controller.start();

    // Wait for resource monitor to finish
    resourceMonitor.stopThread();

    return EXIT_SUCCESS;
}
