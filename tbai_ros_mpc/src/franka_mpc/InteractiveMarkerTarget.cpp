#include "tbai_ros_mpc/franka_mpc/InteractiveMarkerTarget.h"

#include <boost/bind.hpp>

namespace tbai {
namespace mpc {
namespace franka {

InteractiveMarkerTarget::InteractiveMarkerTarget(ros::NodeHandle& nodeHandle,
                                                  const std::string& frameId,
                                                  const Eigen::Vector3d& initialPosition,
                                                  const Eigen::Quaterniond& initialOrientation)
    : server_("franka_ee_target"),
      targetPosition_(initialPosition),
      targetOrientation_(initialOrientation) {
    createInteractiveMarker(frameId, initialPosition, initialOrientation);
}

void InteractiveMarkerTarget::createInteractiveMarker(const std::string& frameId,
                                                       const Eigen::Vector3d& initialPosition,
                                                       const Eigen::Quaterniond& initialOrientation) {
    visualization_msgs::InteractiveMarker intMarker;
    intMarker.header.frame_id = frameId;
    intMarker.header.stamp = ros::Time::now();
    intMarker.name = "ee_target";
    intMarker.description = "End Effector Target";
    intMarker.scale = 0.15;

    // Initial pose
    intMarker.pose.position.x = initialPosition.x();
    intMarker.pose.position.y = initialPosition.y();
    intMarker.pose.position.z = initialPosition.z();
    intMarker.pose.orientation.x = initialOrientation.x();
    intMarker.pose.orientation.y = initialOrientation.y();
    intMarker.pose.orientation.z = initialOrientation.z();
    intMarker.pose.orientation.w = initialOrientation.w();

    // Create a sphere marker for visual feedback
    visualization_msgs::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.scale.x = 0.06;
    sphereMarker.scale.y = 0.06;
    sphereMarker.scale.z = 0.06;
    sphereMarker.color.r = 1.0;
    sphereMarker.color.g = 0.5;
    sphereMarker.color.b = 0.0;
    sphereMarker.color.a = 0.8;

    visualization_msgs::InteractiveMarkerControl sphereControl;
    sphereControl.always_visible = true;
    sphereControl.markers.push_back(sphereMarker);
    intMarker.controls.push_back(sphereControl);

    // Add 6DOF controls
    visualization_msgs::InteractiveMarkerControl control;

    // X-axis (red)
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    // Y-axis (green)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    // Z-axis (blue)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    intMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    intMarker.controls.push_back(control);

    server_.insert(intMarker, boost::bind(&InteractiveMarkerTarget::markerCallback, this, _1));
    server_.applyChanges();

    ROS_INFO("Interactive marker target created at (%.2f, %.2f, %.2f)",
             initialPosition.x(), initialPosition.y(), initialPosition.z());
}

void InteractiveMarkerTarget::markerCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        std::lock_guard<std::mutex> lock(mutex_);

        targetPosition_.x() = feedback->pose.position.x;
        targetPosition_.y() = feedback->pose.position.y;
        targetPosition_.z() = feedback->pose.position.z;

        targetOrientation_.x() = feedback->pose.orientation.x;
        targetOrientation_.y() = feedback->pose.orientation.y;
        targetOrientation_.z() = feedback->pose.orientation.z;
        targetOrientation_.w() = feedback->pose.orientation.w;
    }
}

vector_t InteractiveMarkerTarget::getTargetPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    vector_t pos(3);
    pos << targetPosition_.x(), targetPosition_.y(), targetPosition_.z();
    return pos;
}

vector_t InteractiveMarkerTarget::getTargetOrientation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    vector_t quat(4);
    quat << targetOrientation_.x(), targetOrientation_.y(), targetOrientation_.z(), targetOrientation_.w();
    return quat;
}

void InteractiveMarkerTarget::getTargetPose(vector_t& position, vector_t& orientation) const {
    std::lock_guard<std::mutex> lock(mutex_);

    position.resize(3);
    position << targetPosition_.x(), targetPosition_.y(), targetPosition_.z();

    orientation.resize(4);
    orientation << targetOrientation_.x(), targetOrientation_.y(), targetOrientation_.z(), targetOrientation_.w();
}

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
