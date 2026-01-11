#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlanarTerrainModel.h>
#include <tbai_mpc/quadruped_mpc/terrain/TerrainModel.h>
#include <tbai_ros_ocs2/local_terrain.h>
#include <ros/publisher.h>
#include <ros/ros.h>

namespace tbai::mpc::quadruped {

class LocalTerrainReceiverSynchronizedModule : public ocs2::SolverSynchronizedModule {
   public:
    LocalTerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel> &terrainModel, ros::NodeHandle &nodeHandle);
    ~LocalTerrainReceiverSynchronizedModule() override = default;

    void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                      const ocs2::ReferenceManagerInterface &referenceManager) override;

    void postSolverRun(const ocs2::PrimalSolution &primalSolution) override{};

   private:
    void localTerrainCallback(const tbai_ros_ocs2::local_terrain &localTerrain) {
        vector3_t position;
        position << localTerrain.position[0], localTerrain.position[1], localTerrain.position[2];

        matrix3_t rotation;
        rotation << localTerrain.rotation[0], localTerrain.rotation[1], localTerrain.rotation[2],
            localTerrain.rotation[3], localTerrain.rotation[4], localTerrain.rotation[5], localTerrain.rotation[6],
            localTerrain.rotation[7], localTerrain.rotation[8];

        TerrainPlane terrainPlane(position, rotation);

        {
            std::lock_guard<std::mutex> lock(updateMutex_);
            terrainPtr_ = std::make_unique<PlanarTerrainModel>(terrainPlane);
        }
    }

    std::mutex updateMutex_;
    std::atomic_bool terrainUpdated_;
    std::unique_ptr<PlanarTerrainModel> terrainPtr_;

    ocs2::Synchronized<TerrainModel> *terrainModelPtr_;
    ros::Subscriber localTerrainSubscriber_;
};

}  // namespace tbai::mpc::quadruped
