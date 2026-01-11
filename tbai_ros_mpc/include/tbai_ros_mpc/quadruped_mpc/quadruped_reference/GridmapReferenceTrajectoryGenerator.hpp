#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <tbai_mpc/quadruped_mpc/quadruped_reference/ReferenceTrajectoryGenerator.hpp>
#include <tbai_ros_ocs2/local_terrain.h>

namespace tbai {
namespace mpc {
namespace reference {

/**
 * ROS-dependent reference trajectory generator with gridmap support.
 * Extends the base ReferenceTrajectoryGenerator to use elevation maps when available.
 */
class GridmapReferenceTrajectoryGenerator : public ReferenceTrajectoryGenerator {
   public:
    /**
     * Constructor
     * @param nh: ROS NodeHandle for subscriber setup
     * @param configFile: Path to the target command configuration file
     * @param velocityGeneratorPtr: Reference velocity generator
     * @param kinematicsPtr: Kinematics model for terrain estimation
     * @param trajdt: Time step for reference trajectory
     * @param trajKnots: Number of knots in reference trajectory
     * @param terrainTopic: Topic name for terrain gridmap
     * @param blind: If true, ignores gridmap and uses only terrain plane estimation
     */
    GridmapReferenceTrajectoryGenerator(
        ros::NodeHandle &nh, const std::string &configFile,
        std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
        std::shared_ptr<tbai::mpc::quadruped::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr, ocs2::scalar_t trajdt = 0.1,
        size_t trajKnots = 20, const std::string &terrainTopic = "/elevation_mapping/elevation_map",
        bool blind = false);

    /**
     * Generates reference trajectory, using gridmap if available
     * @param currentTime: Current time
     * @param observation: Current system observation
     * @return Generated target trajectories
     */
    ocs2::TargetTrajectories generateReferenceTrajectory(ocs2::scalar_t currentTime,
                                                         const ocs2::SystemObservation &observation) override;

    /**
     * Checks if gridmap is available
     * @return True if a gridmap has been received
     */
    bool hasGridmap() const;

   private:
    void terrainCallback(const grid_map_msgs::GridMap &msg);
    void publishLocalTerrain(const tbai::mpc::quadruped::TerrainPlane &terrainPlane);

    ros::Subscriber terrainSubscriber_;
    ros::Publisher localTerrainPublisher_;
    std::unique_ptr<grid_map::GridMap> terrainMapPtr_;
    mutable std::mutex terrainMutex_;
    bool blind_;

    // Parameters for gridmap-based reference generation
    ocs2::scalar_t nominalStanceWidthInHeading_ = 0.5;
    ocs2::scalar_t nominalStanceWidthLateral_ = 0.3;
};

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
