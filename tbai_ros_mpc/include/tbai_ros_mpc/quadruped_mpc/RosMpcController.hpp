#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tbai_mpc/quadruped_mpc/MpcController.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>
#include <tbai_ros_mpc/quadruped_mpc/quadruped_reference/GridmapReferenceTrajectoryGenerator.hpp>
#include <tbai_ros_mpc/quadruped_mpc/visualization/QuadrupedVisualizer.h>
#include <tbai_ros_ocs2/MPC_ROS_Interface.hpp>
#include <tbai_ros_ocs2/RosReferenceManager.hpp>
#include <tbai_ros_ocs2/logic/GaitReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/LocalTerrainReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/SwingPlanningVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainPlaneVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainReceiver.h>
#include <tbai_ros_ocs2/terrain/TerrainPlane.h>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {
namespace mpc {

using namespace tbai::mpc::quadruped;

/**
 * Visualizes contact points using ROS markers.
 */
class ContactVisualizer {
   public:
    ContactVisualizer();
    void visualize(const vector_t &currentState, const std::vector<bool> &contacts);

   private:
    std::string odomFrame_;
    ros::Publisher contactPublisher_;
    std::vector<std::string> footFrameNames_;

    pinocchio::Model model_;
    pinocchio::Data data_;
};

/**
 * ROS wrapper for MpcController.
 * Adds ROS-specific visualization and MRT_ROS_Interface support.
 */
class RosMpcController : public tbai::mpc::quadruped::MpcController {
   public:
    /**
     * Constructor
     * @param stateSubscriberPtr: State subscriber for getting robot state
     * @param velocityGeneratorPtr: Reference velocity generator
     */
    RosMpcController(const std::string &robotName, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                     std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
                     std::function<scalar_t()> getCurrentTimeFunction);

    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override { return ros::ok(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    /**
     * Override to create MRT_ROS_Interface and launch ROS nodes.
     * Use useRosInterface parameter to switch between local and ROS-based MPC.
     */
    std::unique_ptr<ocs2::MRT_BASE> createMrtInterface() override;

    /**
     * Override to create MPC_ROS_Interface and launch ROS nodes.
     */
    std::unique_ptr<ocs2::MPC_BASE> createMpcInterface() override;

    /**
     * Sets whether to use MRT_ROS_Interface (for distributed MPC) or MPC_MRT_Interface (for local MPC).
     * @param useRos: If true, uses MRT_ROS_Interface; otherwise uses MPC_MRT_Interface
     */
    void setUseRosInterface(bool useRos) { useRosInterface_ = useRos; }

   protected:
    void referenceThreadLoop() override;

   private:
    void spinOnceReferenceThread();

    // ROS-specific visualization
    std::unique_ptr<tbai::mpc::quadruped::QuadrupedVisualizer> visualizerPtr_;
    std::unique_ptr<ContactVisualizer> contactVisualizerPtr_;
    scalar_t timeSinceLastVisualizationUpdate_ = 1e5;

    // Reference thread ROS infrastructure
    ros::NodeHandle referenceThreadNodeHandle_;
    ros::CallbackQueue referenceThreadCallbackQueue_;

    // Whether to use MRT_ROS_Interface (true) or MPC_MRT_Interface (false)
    bool useRosInterface_ = true;
};

}  // namespace mpc
}  // namespace tbai
