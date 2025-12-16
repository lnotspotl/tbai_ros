#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2w/Go2WRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_go2w/Go2WDreamWaQController.hpp>
#include <tbai_ros_go2w/Go2WStaticController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_go2w_dreamwaq");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_go2w_dreamwaq");
    TBAI_LOG_INFO(logger, "Starting Go2W DreamWaQ deployment node");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize Go2WRobotInterface
    std::shared_ptr<tbai::Go2WRobotInterface> go2wRobotInterface = std::make_shared<tbai::Go2WRobotInterface>(
        tbai::Go2WRobotInterfaceArgs()
            .networkInterface(tbai::getEnvAs<std::string>("TBAI_GO2W_NETWORK_INTERFACE", true, "lo"))
            .unitreeChannel(tbai::getEnvAs<int>("TBAI_GO2W_UNITREE_CHANNEL", true, 1)));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = go2wRobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = go2wRobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    // Create central controller
    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add Go2W-specific static controller for standing
    controller.addController(std::make_unique<tbai::go2w::Go2WStaticController>(stateSubscriber));

    // Get reference velocity generator
    auto referenceGeneratorType = tbai::fromGlobalConfig<std::string>("reference_generator/type");
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> referenceVelocityPtr;

    if (referenceGeneratorType == "twist") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else if (referenceGeneratorType == "joystick") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else {
        TBAI_LOG_WARN(logger, "Unknown reference generator type: {}, using twist", referenceGeneratorType);
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    }

    auto hfRepo = tbai::fromGlobalConfig<std::string>("go2w_controller/hf_repo");
    auto hfModelFolder = tbai::fromGlobalConfig<std::string>("go2w_controller/hf_model_folder");
    auto modelDir = tbai::downloadFromHuggingFace(hfRepo, hfModelFolder);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo, hfModelFolder);

    // Add Go2W DreamWaQ controller
    controller.addController(
        std::make_unique<tbai::go2w::Go2WDreamWaQController>(stateSubscriber, referenceVelocityPtr, modelDir));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
