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

/**
 * @brief Interactive marker for setting end-effector target pose.
 *
 * Creates an interactive marker in RViz that allows the user to drag/rotate
 * the target end-effector pose. The target pose can be read thread-safely
 * from the main control loop.
 */
class InteractiveMarkerTarget {
   public:
    /**
     * @brief Constructor
     * @param nodeHandle ROS node handle
     * @param frameId Frame ID for the marker (e.g., "panda_link0")
     * @param initialPosition Initial target position
     * @param initialOrientation Initial target orientation (x, y, z, w)
     */
    InteractiveMarkerTarget(ros::NodeHandle& nodeHandle,
                            const std::string& frameId,
                            const Eigen::Vector3d& initialPosition,
                            const Eigen::Quaterniond& initialOrientation);

    ~InteractiveMarkerTarget() = default;

    /**
     * @brief Get the current target position (thread-safe)
     * @return Target position as 3D vector
     */
    vector_t getTargetPosition() const;

    /**
     * @brief Get the current target orientation (thread-safe)
     * @return Target orientation as quaternion (x, y, z, w)
     */
    vector_t getTargetOrientation() const;

    /**
     * @brief Get both position and orientation atomically
     * @param position Output: target position (3D)
     * @param orientation Output: target orientation (x, y, z, w)
     */
    void getTargetPose(vector_t& position, vector_t& orientation) const;

   private:
    void createInteractiveMarker(const std::string& frameId,
                                  const Eigen::Vector3d& initialPosition,
                                  const Eigen::Quaterniond& initialOrientation);

    void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // Interactive marker server
    interactive_markers::InteractiveMarkerServer server_;

    // Thread-safe target pose storage
    mutable std::mutex mutex_;
    Eigen::Vector3d targetPosition_;
    Eigen::Quaterniond targetOrientation_;
};

}  // namespace tbai::mpc::arm
