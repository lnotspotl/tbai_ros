#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ocs2_core/Types.h>

namespace tbai::mpc::arm {

using ocs2::scalar_t;
using ocs2::vector_t;

class InteractiveMarkerTarget {
   public:

    InteractiveMarkerTarget(ros::NodeHandle& nodeHandle,
                            const std::string& frameId,
                            const Eigen::Vector3d& initialPosition,
                            const Eigen::Quaterniond& initialOrientation);

    ~InteractiveMarkerTarget() = default;

    vector_t getTargetPosition() const;


    vector_t getTargetOrientation() const;


    void getTargetPose(vector_t& position, vector_t& orientation) const;

   private:
    void createInteractiveMarker(const std::string& frameId,
                                  const Eigen::Vector3d& initialPosition,
                                  const Eigen::Quaterniond& initialOrientation);

    void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    interactive_markers::InteractiveMarkerServer server_;

    mutable std::mutex mutex_;
    Eigen::Vector3d targetPosition_;
    Eigen::Quaterniond targetOrientation_;
};

}  // namespace tbai::mpc::arm
