#include "tbai_ros_ocs2/quadruped_interface/LocalTerrainReceiver.h"

namespace switched_model {

LocalTerrainReceiverSynchronizedModule::LocalTerrainReceiverSynchronizedModule(
    ocs2::Synchronized<TerrainModel> &terrainModel, ros::NodeHandle &nodeHandle)
    : terrainModelPtr_(&terrainModel) {
    localTerrainSubscriber_ =
        nodeHandle.subscribe("local_terrain", 1, &LocalTerrainReceiverSynchronizedModule::localTerrainCallback, this);
}

void LocalTerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                          const vector_t &currentState,
                                                          const ocs2::ReferenceManagerInterface &referenceManager) {
    std::lock_guard<std::mutex> lock(updateMutex_);
    if (terrainPtr_) {
        terrainModelPtr_->reset(std::move(terrainPtr_));
    }
}

}  // namespace switched_model
