// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_mpc/RosMpcController.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/anymal_interface/Interface.h>
#include <tbai_mpc/wbc/Factory.hpp>
#include <tbai_ros_ocs2/MRT_ROS_Interface.hpp>
#include <visualization_msgs/Marker.h>

namespace tbai {
namespace mpc {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ContactVisualizer::ContactVisualizer() {
    ros::NodeHandle nh;
    contactPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/contact_points", 1);

    odomFrame_ = "odom";
    footFrameNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // Setup Pinocchio model
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ContactVisualizer::visualize(const vector_t& currentState, const std::vector<bool>& contacts) {
    ros::Time timeStamp = ros::Time::now();

    vector_t q = vector_t::Zero(model_.nq);
    q.head<3>() = currentState.segment<3>(3);                               // Position
    q.segment<4>(3) = tbai::ocs2rpy2quat(currentState.head<3>()).coeffs();  // Orientation
    q.tail(12) = currentState.segment<12>(3 + 3 + 3 + 3);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < footFrameNames_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = timeStamp;
        marker.header.frame_id = odomFrame_;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = contacts[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.pose.position.x = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[0];
        marker.pose.position.y = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[1];
        marker.pose.position.z = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        markerArray.markers.push_back(marker);
    }
    contactPublisher_.publish(markerArray);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosMpcController::RosMpcController(const std::shared_ptr<tbai::StateSubscriber>& stateSubscriberPtr,
                                   std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr)
    : MpcController(stateSubscriberPtr, std::move(velocityGeneratorPtr)) {
    using tbai::fromGlobalConfig;

    initTime_ = tbai::readInitTime();

    ros::NodeHandle nh;

    // Load configuration file paths from ROS param server
    std::string targetCommandConfig;
    TBAI_THROW_UNLESS(nh.getParam("/target_command_config_file", targetCommandConfig),
                      "Failed to get parameter /target_command_config_file");

    std::string urdfString;
    TBAI_THROW_UNLESS(nh.getParam("/robot_description", urdfString), "Failed to get parameter /robot_description");

    std::string taskSettingsFile;
    TBAI_THROW_UNLESS(nh.getParam("/task_settings_file", taskSettingsFile),
                      "Failed to get parameter /task_settings_file");

    std::string frameDeclarationFile;
    TBAI_THROW_UNLESS(nh.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get parameter /frame_declaration_file");

    std::string controllerConfigFile;
    TBAI_THROW_UNLESS(nh.getParam("/controller_config_file", controllerConfigFile),
                      "Failed to get parameter /controller_config_file");

    // Load reference trajectory settings
    scalar_t trajdt = fromGlobalConfig<scalar_t>("mpc_controller/reference_trajectory/traj_dt");
    size_t trajKnots = fromGlobalConfig<size_t>("mpc_controller/reference_trajectory/traj_knots");
    bool blind = fromGlobalConfig<bool>("mpc_controller/reference_trajectory/blind");

    // Initialize base controller
    initialize(urdfString, taskSettingsFile, frameDeclarationFile, controllerConfigFile, targetCommandConfig, trajdt,
               trajKnots);

    // Create ROS-specific components
    visualizerPtr_ = std::make_unique<switched_model::QuadrupedVisualizer>(quadrupedInterfacePtr_->getKinematicModel(),
                                                                           quadrupedInterfacePtr_->getJointNames(),
                                                                           quadrupedInterfacePtr_->getBaseName(), nh);
    contactVisualizerPtr_ = std::make_unique<ContactVisualizer>();

    // Replace the base reference trajectory generator with gridmap-aware version
    referenceThreadNodeHandle_.setCallbackQueue(&referenceThreadCallbackQueue_);
    auto kinematicsPtr = std::shared_ptr<switched_model::KinematicsModelBase<scalar_t>>(
        quadrupedInterfacePtr_->getKinematicModel().clone());
    auto terrainTopic = fromGlobalConfig<std::string>("mpc_controller/reference_trajectory/terrain_topic");
    referenceTrajectoryGeneratorPtr_ = std::make_unique<reference::GridmapReferenceTrajectoryGenerator>(
        referenceThreadNodeHandle_, targetCommandConfig, velocityGeneratorPtr_, std::move(kinematicsPtr), trajdt,
        trajKnots, terrainTopic, blind);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::unique_ptr<ocs2::MRT_BASE> RosMpcController::createMrtInterface() {
    if (useRosInterface_) {
        // Create MRT_ROS_Interface for distributed MPC
        auto mrtRos = std::make_unique<tbai::ocs2_ros::MRT_ROS_Interface>("anymal");
        ros::NodeHandle nh;
        mrtRos->launchNodes(nh);
        return mrtRos;
    } else {
        // Use base implementation (MPC_MRT_Interface)
        return MpcController::createMrtInterface();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosMpcController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();

    // For MRT_ROS_Interface, also spin the MRT callback queue
    if (useRosInterface_) {
        auto* mrtRos = dynamic_cast<tbai::ocs2_ros::MRT_ROS_Interface*>(mrtPtr_.get());
        if (mrtRos != nullptr) {
            mrtRos->spinMRT();
        }
    }

    state_ = stateSubscriberPtr_->getLatestState();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosMpcController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 15.0) {
        visualizerPtr_->update(generateSystemObservation(), mrtPtr_->getPolicy(), mrtPtr_->getCommand());
        contactVisualizerPtr_->visualize(state_.x, state_.contactFlags);
        timeSinceLastVisualizationUpdate_ = 0.0;
    }
    timeSinceLastVisualizationUpdate_ += dt;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosMpcController::spinOnceReferenceThread() {
    referenceThreadCallbackQueue_.callAvailable(ros::WallDuration(0.0));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosMpcController::referenceThreadLoop() {
    referenceTrajectoryGeneratorPtr_->reset();

    // Wait for initial observation
    while (ros::ok() && !stopReferenceThread_) {
        spinOnceReferenceThread();
        if (referenceTrajectoryGeneratorPtr_->isInitialized()) break;
        ros::Duration(0.02).sleep();
    }

    // Reference loop
    ros::Rate rate(referenceThreadRate_);
    while (ros::ok() && !stopReferenceThread_) {
        spinOnceReferenceThread();

        // Generate and publish reference trajectory
        auto observation = generateSystemObservation();
        auto targetTrajectories =
            referenceTrajectoryGeneratorPtr_->generateReferenceTrajectory(tNow_, observation);
        mrtPtr_->setTargetTrajectories(targetTrajectories);

        TBAI_LOG_INFO_THROTTLE(logger_, 5.0, "Publishing reference");
        rate.sleep();
    }
}

}  // namespace mpc
}  // namespace tbai
