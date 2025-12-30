// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_dtc/RosDtcController.hpp"

#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_ocs2/common/RosMsgConversions.h>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace dtc {

RosDtcController::RosDtcController(const std::string &robotName,
                                   const std::shared_ptr<tbai::StateSubscriber> &stateSubscriber,
                                   std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGenerator,
                                   std::function<scalar_t()> getCurrentTimeFunction)
    : DtcController(robotName, stateSubscriber, std::move(velocityGenerator), std::move(getCurrentTimeFunction)) {
    // Load parameters from ROS parameter server
    std::string taskFile, urdfFile, referenceFile;
    TBAI_THROW_UNLESS(nh_.getParam("taskFile", taskFile), "Task file not found");
    TBAI_THROW_UNLESS(nh_.getParam("urdfFile", urdfFile), "URDF file not found");
    TBAI_THROW_UNLESS(nh_.getParam("referenceFile", referenceFile), "Reference file not found");

    // Get URDF from parameter server
    std::string urdf, taskSettingsFile, frameDeclarationFile;
    ros::param::get("/robot_description", urdf);
    ros::param::get("/task_settings_file", taskSettingsFile);
    ros::param::get("/frame_declaration_file", frameDeclarationFile);

    // Load DTC model
    std::string hfRepo = tbai::fromGlobalConfig<std::string>("dtc_controller/hf_repo");
    std::string hfModel = tbai::fromGlobalConfig<std::string>("dtc_controller/hf_model");
    std::string modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);

    // Initialize reference publisher
    refPub_ = nh_.advertise<tbai_ros_ocs2::mpc_target_trajectories>(robotName_ + "_mpc_target", 1, false);

    // Check if blind mode
    bool blind = tbai::fromGlobalConfig<bool>("dtc_controller/blind");
    if (!blind) {
        TBAI_LOG_INFO(logger_, "Initializing gridmap interface for perceptive mode");
        setTerrainInterface(std::make_unique<GridmapTerrainInterface>());
    } else {
        TBAI_LOG_INFO(logger_, "Running in blind mode");
    }

    // Initialize the base controller
    initialize(urdf, taskSettingsFile, frameDeclarationFile, modelPath);

    // Initialize visualizer after quadruped interface is ready
    visualizer_.reset(new switched_model::QuadrupedVisualizer(quadrupedInterface_->getKinematicModel(),
                                                              quadrupedInterface_->getJointNames(),
                                                              quadrupedInterface_->getBaseName(), nh_));
}

std::unique_ptr<ocs2::MRT_BASE> RosDtcController::createMrtInterface() {
    // Create ROS-based MRT interface
    auto mrtRos = std::make_unique<tbai::ocs2_ros::MRT_ROS_Interface>(robotName_);
    mrtRos->launchNodes(nh_);
    return mrtRos;
}

std::unique_ptr<ocs2::MPC_BASE> RosDtcController::createMpcInterface() {
    // MPC runs in separate node when using ROS interface
    return nullptr;
}

void RosDtcController::publishReference(const TargetTrajectories &targetTrajectories) {
    const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
    refPub_.publish(mpcTargetTrajectoriesMsg);
}

void RosDtcController::postStep(scalar_t currentTime, scalar_t dt) {
    visualizer_->update(generateSystemObservation(), mrt_->getPolicy(), mrt_->getCommand());
}

void RosDtcController::resetMpc() {
    TBAI_LOG_INFO(logger_, "Waiting for state subscriber to initialize...");

    // Wait to receive observation
    stateSubscriberPtr_->waitTillInitialized();

    TBAI_LOG_INFO(logger_, "State subscriber initialized");

    // Prepare initial observation for MPC
    ocs2::SystemObservation mpcObservation = generateSystemObservation();

    TBAI_LOG_INFO(logger_, "Initial observation generated");

    // Prepare target trajectory
    ocs2::TargetTrajectories initTargetTrajectories({0.0}, {mpcObservation.state}, {mpcObservation.input});

    TBAI_LOG_INFO(logger_, "Resetting MPC...");

    // Use the ROS-specific reset
    auto *mrtRos = dynamic_cast<tbai::ocs2_ros::MRT_ROS_Interface *>(mrt_.get());
    if (mrtRos != nullptr) {
        mrtRos->resetMpcNode(initTargetTrajectories);

        while (!mrt_->initialPolicyReceived() && ros::ok()) {
            TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
            ros::spinOnce();
            mrtRos->spinMRT();
            mrt_->setCurrentObservation(generateSystemObservation());
            ros::Duration(0.1).sleep();
        }
    }

    TBAI_LOG_INFO(logger_, "Initial policy received");
}

}  // namespace dtc
}  // namespace tbai
