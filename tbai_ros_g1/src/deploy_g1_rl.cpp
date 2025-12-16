#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_g1/G1RLController.hpp>
#include <tbai_ros_g1/G1MimicController.hpp>
#include <tbai_ros_g1/G1StaticController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <ros/package.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_g1_rl");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_g1_rl");
    TBAI_LOG_INFO(logger, "Starting G1 RL deployment node");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize G1RobotInterface
    std::shared_ptr<tbai::G1RobotInterface> g1RobotInterface =
        std::make_shared<tbai::G1RobotInterface>(
            tbai::G1RobotInterfaceArgs()
                .networkInterface(tbai::getEnvAs<std::string>("TBAI_G1_NETWORK_INTERFACE", true, "lo"))
                .unitreeChannel(tbai::getEnvAs<int>("TBAI_G1_UNITREE_CHANNEL", true, 1)));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = g1RobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = g1RobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add G1-specific static controller for standing (handles 29 DOF)
    controller.addController(std::make_unique<tbai::g1::G1StaticController>(stateSubscriber));

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

    // Load G1 walking controller
    auto hfRepo = tbai::fromGlobalConfig<std::string>("g1_controller/hf_repo");
    auto hfModel = tbai::fromGlobalConfig<std::string>("g1_controller/hf_model");
    auto modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo, hfModel);
    controller.addController(
        std::make_unique<tbai::g1::G1RLController>(stateSubscriber, referenceVelocityPtr, modelPath));

    // Load G1 mimic dance 102 controller
    auto hfRepo3 = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_repo");
    auto hfModel3 = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_model");
    auto motionFps = tbai::fromGlobalConfig<float>("g1_mimic_dance102/motion_fps");
    auto timeStart = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_start");
    auto timeEnd = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_end");
    auto dance102MotionFile = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_motion_file");
    auto modelPathDance102 = tbai::downloadFromHuggingFace(hfRepo3, hfModel3);
    auto dance102MotionFilePath = tbai::downloadFromHuggingFace(hfRepo3, dance102MotionFile);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo3, hfModel3);
    controller.addController(
        std::make_unique<tbai::g1::G1MimicController>(stateSubscriber, modelPathDance102, dance102MotionFilePath, motionFps, timeStart, timeEnd, "G1MimicDance102"));

    // Load G1 mimic gangnam style controller
    auto hfRepo2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_repo");
    auto hfModel2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_model");
    auto motionFps2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/motion_fps");
    auto timeStart2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_start");
    auto timeEnd2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_end");
    auto gangnamMotionFile2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_motion_file");
    auto modelPathGangnam = tbai::downloadFromHuggingFace(hfRepo2, hfModel2);
    auto gangnamMotionFilePath = tbai::downloadFromHuggingFace(hfRepo2, gangnamMotionFile2);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo2, hfModel2);
    controller.addController(
            std::make_unique<tbai::g1::G1MimicController>(stateSubscriber, modelPathGangnam, gangnamMotionFilePath, motionFps2, timeStart2, timeEnd2, "G1MimicGangnam"));
        
    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
