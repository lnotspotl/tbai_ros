#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

namespace tbai {
namespace mpc {
namespace franka {

using ocs2::scalar_t;
using ocs2::vector_t;

class FrankaVisualizer {
   public:
    FrankaVisualizer(ros::NodeHandle& nodeHandle,
                     const std::vector<std::string>& jointNames,
                     scalar_t maxUpdateFrequency = 30.0);
    ~FrankaVisualizer() = default;

    void update(const vector_t& jointPositions,
                const vector_t& currentEEPosition,
                const vector_t& currentEEOrientation,
                const vector_t& targetEEPosition,
                const vector_t& targetEEOrientation,
                const std::vector<vector_t>& eeTrajectory,
                const ocs2::SystemObservation& observation,
                const ocs2::PrimalSolution& primalSolution);

   private:
    void launchNode(ros::NodeHandle& nodeHandle);

    void publishRobotState(ros::Time timeStamp, const vector_t& jointPositions);

    void publishCurrentEEMarker(ros::Time timeStamp,
                                 const vector_t& position,
                                 const vector_t& orientation);

    void publishTargetEEMarker(ros::Time timeStamp,
                                const vector_t& position,
                                const vector_t& orientation);

    void publishDesiredEETrajectory(ros::Time timeStamp,
                                     const std::vector<vector_t>& eeTrajectory);

    // Robot state publisher
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    std::vector<std::string> jointNames_;

    // TF broadcaster
    tf::TransformBroadcaster tfBroadcaster_;

    // Publishers
    ros::Publisher currentEEPublisher_;
    ros::Publisher targetEEPublisher_;
    ros::Publisher desiredEETrajectoryPublisher_;
    ros::Publisher targetPosePublisher_;

    // Settings
    std::string frameId_ = "panda_link0";
    std::string baseFrame_ = "panda_link0";
    scalar_t eeMarkerDiameter_ = 0.05;
    scalar_t trajectoryLineWidth_ = 0.01;

    // Rate limiting
    scalar_t lastTime_;
    scalar_t minPublishTimeDifference_;
};

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
