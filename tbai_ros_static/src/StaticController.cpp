// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_static/StaticController.hpp"

#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <ros/package.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_ros_msgs/EstimatedState.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {
namespace static_ {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
RosStaticController::RosStaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr)
    : StaticController(stateSubscriberPtr) {
    // Setup state publisher
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    KDL::Tree kdlTree;
    kdl_parser::treeFromString(urdfString, kdlTree);
    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    // Setup pinocchio model
    setupPinocchioModel();
    footFrameNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    publishState_ = tbai::fromGlobalConfig<bool>("static_controller/publish_state", false);
    TBAI_LOG_INFO(logger_, "Controller state publishing is {}", publishState_ ? "enabled" : "disabled");
    if (publishState_) {
        statePublisher_ = ros::NodeHandle().advertise<tbai_ros_msgs::EstimatedState>(
            tbai::fromGlobalConfig<std::string>("static/state_topic", "estimated_state"), 10);
    }

    baseFrameName_ = tbai::fromGlobalConfig<std::string>("base_name");

    // Some dummy value
    timeSinceLastVisualizationUpdate_ = 1000.0;

    // Setup contact point publisher
    ros::NodeHandle nh_local;
    contactPointPublisher_ = nh_local.advertise<visualization_msgs::MarkerArray>("/contact_points", 10);
}
/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::setupPinocchioModel() {
    // Get URDF string
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    if (fixedBase_) {
        pinocchio::urdf::buildModelFromXML(urdfString, model_);
    } else {
        pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), model_);
    }
    data_ = pinocchio::Data(model_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        auto currentRosTime = ros::Time::now();
        publishOdomBaseTransforms(state_.x, currentRosTime);
        publishJointAngles(state_.x, currentRosTime);
        if (!fixedBase_) {
            visualizeContactPoints(state_.x, state_.contactFlags, currentRosTime);
        }
        if (publishState_) {
            publishEstimatedState();
        }
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime) {
    geometry_msgs::TransformStamped odomBaseTransform;

    // Header
    odomBaseTransform.header.stamp = currentTime;
    odomBaseTransform.header.frame_id = "odom";
    odomBaseTransform.child_frame_id = baseFrameName_;

    if (fixedBase_) {
        // Fixed base: publish identity transform
        odomBaseTransform.transform.translation.x = 0.0;
        odomBaseTransform.transform.translation.y = 0.0;
        odomBaseTransform.transform.translation.z = 0.0;
        odomBaseTransform.transform.rotation.x = 0.0;
        odomBaseTransform.transform.rotation.y = 0.0;
        odomBaseTransform.transform.rotation.z = 0.0;
        odomBaseTransform.transform.rotation.w = 1.0;
    } else {
        // Position
        odomBaseTransform.transform.translation.x = currentState(3);
        odomBaseTransform.transform.translation.y = currentState(4);
        odomBaseTransform.transform.translation.z = currentState(5);

        // Orientation
        tbai::quaternion_t quat = tbai::ocs2rpy2quat(currentState.head<3>());
        odomBaseTransform.transform.rotation.x = quat.x();
        odomBaseTransform.transform.rotation.y = quat.y();
        odomBaseTransform.transform.rotation.z = quat.z();
        odomBaseTransform.transform.rotation.w = quat.w();
    }

    // Publish
    tfBroadcaster_.sendTransform(odomBaseTransform);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::publishJointAngles(const vector_t &currentState, const ros::Time &currentTime) {
    std::map<std::string, scalar_t> jointPositionMap;
    const size_t offset = fixedBase_ ? 0 : (3 + 3 + 3 + 3);
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        jointPositionMap[jointNames_[i]] = currentState(i + offset);
    }
    robotStatePublisherPtr_->publishTransforms(jointPositionMap, currentTime);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::publishEstimatedState() {
    tbai_ros_msgs::EstimatedState stateMsg;
    stateMsg.timestamp = state_.timestamp;

    // Resize dynamic arrays
    const size_t baseStateSize = fixedBase_ ? 0 : 12;
    const size_t numJoints = (state_.x.size() - baseStateSize) / 2;  // state = base + n_joints + n_joints
    stateMsg.joint_angles.resize(numJoints);
    stateMsg.joint_velocities.resize(numJoints);
    stateMsg.contact_flags.resize(state_.contactFlags.size());

    if (!fixedBase_) {
        // Base position
        std::copy(state_.x.data() + 3, state_.x.data() + 3 + 3, stateMsg.base_position.begin());  // position

        // Base orientation
        tbai::quaternion_t quat = tbai::ocs2rpy2quat(state_.x.head<3>());
        std::copy(quat.coeffs().data(), quat.coeffs().data() + 4, stateMsg.base_orientation_xyzw.begin());  // orientation

        // Base linear velocity
        std::copy(state_.x.data() + 3 + 3 + 3, state_.x.data() + 3 + 3 + 3 + 3,
                  stateMsg.base_lin_vel.begin());  // linear velocity

        // Base angular velocity
        std::copy(state_.x.data() + 3 + 3, state_.x.data() + 3 + 3 + 3, stateMsg.base_ang_vel.begin());  // angular velocity
    }

    // Joint positions
    std::copy(state_.x.data() + baseStateSize, state_.x.data() + baseStateSize + numJoints, stateMsg.joint_angles.begin());

    // Joint velocities
    std::copy(state_.x.data() + baseStateSize + numJoints, state_.x.data() + baseStateSize + numJoints + numJoints,
              stateMsg.joint_velocities.begin());

    // Contact flags
    std::copy(state_.contactFlags.begin(), state_.contactFlags.end(), stateMsg.contact_flags.begin());
    statePublisher_.publish(stateMsg);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RosStaticController::visualizeContactPoints(const vector_t &currentState, const std::vector<bool> &contacts,
                                                 const ros::Time &currentTime) {
    vector_t q = vector_t::Zero(model_.nq);
    q.head<3>() = currentState.segment<3>(3);                               // Position
    q.segment<4>(3) = tbai::ocs2rpy2quat(currentState.head<3>()).coeffs();  // Orientation
    q.tail(12) = currentState.segment<12>(3 + 3 + 3 + 3);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    // publish sphere markers for each contact point
    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < footFrameNames_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = currentTime;
        marker.header.frame_id = "odom";
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
    contactPointPublisher_.publish(markerArray);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
}  // namespace static_
}  // namespace tbai
