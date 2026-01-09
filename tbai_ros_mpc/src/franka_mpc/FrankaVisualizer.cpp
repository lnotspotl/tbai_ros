#include "tbai_ros_mpc/franka_mpc/FrankaVisualizer.h"

#include <limits>
#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tbai_ros_ocs2/visualization/VisualizationHelpers.h>
#include <urdf/model.h>

namespace tbai {
namespace mpc {
namespace franka {

FrankaVisualizer::FrankaVisualizer(ros::NodeHandle &nodeHandle, const std::vector<std::string> &jointNames,
                                   scalar_t maxUpdateFrequency)
    : jointNames_(jointNames),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
    // Setup robot state publisher from URDF
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        ROS_ERROR("Failed to get param /robot_description for FrankaVisualizer");
    } else {
        KDL::Tree kdlTree;
        if (kdl_parser::treeFromString(urdfString, kdlTree)) {
            robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
            robotStatePublisherPtr_->publishFixedTransforms(true);
        } else {
            ROS_ERROR("Failed to parse URDF for robot state publisher");
        }
    }

    launchNode(nodeHandle);
}

void FrankaVisualizer::launchNode(ros::NodeHandle &nodeHandle) {
    currentEEPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/franka/currentEE", 1);
    targetEEPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/franka/targetEE", 1);
    desiredEETrajectoryPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/franka/desiredEETrajectory", 1);
    targetPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("/franka/targetPose", 1);
}

void FrankaVisualizer::update(const vector_t &jointPositions, const vector_t &currentEEPosition,
                              const vector_t &currentEEOrientation, const vector_t &targetEEPosition,
                              const vector_t &targetEEOrientation, const std::vector<vector_t> &eeTrajectory,
                              const ocs2::SystemObservation &observation, const ocs2::PrimalSolution &primalSolution) {
    if (observation.time - lastTime_ < minPublishTimeDifference_) {
        return;
    }

    const auto timeStamp = ros::Time::now();

    // Publish robot state TF (joint angles)
    publishRobotState(timeStamp, jointPositions);

    // Publish visualization markers
    publishCurrentEEMarker(timeStamp, currentEEPosition, currentEEOrientation);
    publishTargetEEMarker(timeStamp, targetEEPosition, targetEEOrientation);
    publishDesiredEETrajectory(timeStamp, eeTrajectory);

    lastTime_ = observation.time;
}

void FrankaVisualizer::publishRobotState(ros::Time timeStamp, const vector_t &jointPositions) {
    if (!robotStatePublisherPtr_) {
        return;
    }

    // Publish base transform (identity for fixed-base manipulator)
    geometry_msgs::TransformStamped baseTransform;
    baseTransform.header.stamp = timeStamp;
    baseTransform.header.frame_id = "world";
    baseTransform.child_frame_id = baseFrame_;
    baseTransform.transform.translation.x = 0.0;
    baseTransform.transform.translation.y = 0.0;
    baseTransform.transform.translation.z = 0.0;
    baseTransform.transform.rotation.x = 0.0;
    baseTransform.transform.rotation.y = 0.0;
    baseTransform.transform.rotation.z = 0.0;
    baseTransform.transform.rotation.w = 1.0;
    tfBroadcaster_.sendTransform(baseTransform);

    // Build joint position map
    std::map<std::string, scalar_t> jointPositionMap;
    for (size_t i = 0; i < jointNames_.size() && i < static_cast<size_t>(jointPositions.size()); ++i) {
        jointPositionMap[jointNames_[i]] = jointPositions(i);
    }

    // Publish joint transforms
    robotStatePublisherPtr_->publishTransforms(jointPositionMap, timeStamp);
}

void FrankaVisualizer::publishCurrentEEMarker(ros::Time timeStamp, const vector_t &position,
                                              const vector_t &orientation) {
    Eigen::Vector3d pos(position(0), position(1), position(2));

    visualization_msgs::Marker marker = ocs2::getSphereMsg(pos, ocs2::Color::blue, eeMarkerDiameter_);
    marker.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    marker.ns = "Current EE";
    marker.id = 0;

    currentEEPublisher_.publish(marker);
}

void FrankaVisualizer::publishTargetEEMarker(ros::Time timeStamp, const vector_t &position,
                                             const vector_t &orientation) {
    Eigen::Vector3d pos(position(0), position(1), position(2));
    Eigen::Quaterniond quat(orientation(3), orientation(0), orientation(1), orientation(2));  // w, x, y, z

    // Sphere marker for target position
    visualization_msgs::Marker marker = ocs2::getSphereMsg(pos, ocs2::Color::green, eeMarkerDiameter_);
    marker.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    marker.ns = "Target EE";
    marker.id = 0;

    targetEEPublisher_.publish(marker);

    // Also publish as PoseStamped for easier debugging
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    poseMsg.pose.position = ocs2::getPointMsg(pos);
    poseMsg.pose.orientation = ocs2::getOrientationMsg(quat);

    targetPosePublisher_.publish(poseMsg);
}

void FrankaVisualizer::publishDesiredEETrajectory(ros::Time timeStamp, const std::vector<vector_t> &eeTrajectory) {
    if (eeTrajectory.empty()) {
        return;
    }

    // Create line strip from trajectory points
    std::vector<geometry_msgs::Point> points;
    points.reserve(eeTrajectory.size());

    for (const auto &eePos : eeTrajectory) {
        Eigen::Vector3d pos(eePos(0), eePos(1), eePos(2));
        points.push_back(ocs2::getPointMsg(pos));
    }

    visualization_msgs::Marker lineMarker;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.points = std::move(points);
    lineMarker.scale.x = trajectoryLineWidth_;
    lineMarker.color = ocs2::getColor(ocs2::Color::yellow);
    lineMarker.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    lineMarker.ns = "EE Trajectory";
    lineMarker.id = 0;
    lineMarker.pose.orientation.w = 1.0;

    desiredEETrajectoryPublisher_.publish(lineMarker);
}

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
