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

#include <tbai_ros_ocs2/MPC_ROS_Interface.hpp>
#include <tbai_ros_ocs2/RosReferenceManager.hpp>

#include <tbai_ros_ocs2/logic/GaitReceiver.h>
#include <tbai_ros_ocs2/terrain/TerrainPlane.h>

#include <tbai_ros_ocs2/quadruped_interface/SwingPlanningVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainPlaneVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/LocalTerrainReceiver.h>

#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>

namespace tbai {
namespace mpc {

using namespace switched_model;

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

    std::unique_ptr<ocs2::MPC_BASE> createMpcInterface() override {

        ros::NodeHandle nodeHandle;

        // Prepare robot interface
        // const std::string taskSettingsFile = "/home/kb/Documents/ros/src/tbai_ros/tbai_ros_mpc/config/anymal_d/perceptive/task.info";
        const std::string taskSettingsFile = "/home/kb/Documents/ros/src/tbai_ros/tbai_ros_mpc/config/go2/blind/task.info";
        const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

        // const std::string sqpSettingsFile = "/home/kb/Documents/ros/src/tbai_ros/tbai_ros_mpc/config/anymal_d/perceptive/sqp.info";
        const std::string sqpSettingsFile = "/home/kb/Documents/ros/src/tbai_ros/tbai_ros_mpc/config/go2/blind/sqp.info";
        const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
        auto mpcPtr = switched_model::getSqpMpc(*quadrupedInterfacePtr_, mpcSettings, sqpSettings);

        const std::string robotName = "go2";

        QuadrupedInterface& quadrupedInterface = *quadrupedInterfacePtr_;

        auto solverModules = quadrupedInterface.getSynchronizedModules();
      
        // Gait
        auto gaitReceiver =
            std::make_shared<GaitReceiver>(nodeHandle, quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
        solverModules.push_back(gaitReceiver);
      
        // Terrain Receiver
        auto terrainReceiver = std::make_shared<TerrainReceiverSynchronizedModule>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nodeHandle);
        solverModules.push_back(terrainReceiver);
      
        // Local Terrain Receiver
        auto localTerrainReceiver = std::make_shared<LocalTerrainReceiverSynchronizedModule>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nodeHandle);
        solverModules.push_back(localTerrainReceiver);
      
        // Terrain plane visualization
        auto terrainVisualizer = std::make_shared<TerrainPlaneVisualizerSynchronizedModule>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
        solverModules.push_back(terrainVisualizer);
      
        // Swing planner
        auto swingPlanningVisualizer = std::make_shared<SwingPlanningVisualizer>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
        solverModules.push_back(swingPlanningVisualizer);
      
        // reference manager
        auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(nodeHandle);
        mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
      
        // MPC
        mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);

        return mpcPtr;
    }

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
    bool useRosInterface_ = false;
};

}  // namespace mpc
}  // namespace tbai
