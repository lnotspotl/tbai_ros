#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>

#include <ros/ros.h>
#include <tbai_joe/JoeController.hpp>
#include <tbai_ros_gridmap/GridmapInterface.hpp>
#include <tbai_ros_mpc/quadruped_mpc/visualization/QuadrupedVisualizer.h>
#include <tbai_ros_ocs2/MRT_ROS_Interface.hpp>
#include <tbai_ros_ocs2/mpc_target_trajectories.h>

namespace tbai {
namespace joe {

/**
 * Gridmap-based terrain interface implementation for JOE.
 */
class GridmapTerrainInterface : public TerrainInterface {
   public:
    GridmapTerrainInterface() : gridmap_(tbai::gridmap::getGridmapInterfaceUnique()) {}

    scalar_t atPosition(scalar_t x, scalar_t y) const override { return gridmap_->atPosition(x, y); }

    bool isInitialized() const override { return gridmap_->isInitialized(); }

    void waitTillInitialized() override { gridmap_->waitTillInitialized(); }

    TargetTrajectories generateTargetTrajectories(
        scalar_t currentTime, const BaseReferenceHorizon &horizon, const BaseReferenceState &state,
        const BaseReferenceCommand &command, const switched_model::QuadrupedInterface &quadrupedInterface) override;

    tbai::gridmap::GridmapInterface &getGridmap() { return *gridmap_; }

   private:
    std::unique_ptr<tbai::gridmap::GridmapInterface> gridmap_;
};

/**
 * ROS wrapper for JoeController.
 * Adds ROS-specific visualization, MRT_ROS_Interface, and gridmap support.
 */
class RosJoeController : public JoeController {
   public:
    RosJoeController(const std::string &robotName, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriber,
                     std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGenerator,
                     std::function<scalar_t()> getCurrentTimeFunction);

    bool ok() const override { return ros::ok(); }

    void preStep(scalar_t currentTime, scalar_t dt) override {
        ros::spinOnce();
        JoeController::preStep(currentTime, dt);
    }

    void postStep(scalar_t currentTime, scalar_t dt) override;

    std::unique_ptr<ocs2::MRT_BASE> createMrtInterface() override;

    std::unique_ptr<ocs2::MPC_BASE> createMpcInterface() override;

   protected:
    void publishReference(const TargetTrajectories &targetTrajectories) override;

    void resetMpc() override;

   private:
    ros::NodeHandle nh_;
    ros::Publisher refPub_;
    std::unique_ptr<switched_model::QuadrupedVisualizer> visualizer_;
};

}  // namespace joe
}  // namespace tbai
