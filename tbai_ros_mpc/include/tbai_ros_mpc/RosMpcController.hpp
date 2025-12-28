#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>

#include <tbai_ros_mpc/visualization/QuadrupedVisualizer.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tbai_mpc/MpcController.hpp>
#include <tbai_ros_mpc/reference/GridmapReferenceTrajectoryGenerator.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {
namespace mpc {

/**
 * Visualizes contact points using ROS markers.
 */
class ContactVisualizer {
   public:
    ContactVisualizer();
    void visualize(const vector_t& currentState, const std::vector<bool>& contacts);

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
class RosMpcController : public MpcController {
   public:
    /**
     * Constructor
     * @param stateSubscriberPtr: State subscriber for getting robot state
     * @param velocityGeneratorPtr: Reference velocity generator
     */
    RosMpcController(const std::shared_ptr<tbai::StateSubscriber>& stateSubscriberPtr,
                     std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr);

    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override { return ros::ok(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    /**
     * Override to create MRT_ROS_Interface and launch ROS nodes.
     * Use useRosInterface parameter to switch between local and ROS-based MPC.
     */
    std::unique_ptr<ocs2::MRT_BASE> createMrtInterface() override;

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
    std::unique_ptr<switched_model::QuadrupedVisualizer> visualizerPtr_;
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
