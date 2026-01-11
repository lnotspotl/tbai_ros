//
// Created by rgrandia on 28.09.20.
//

#include "tbai_ros_ocs2/quadruped_interface/TerrainReceiver.h"

namespace tbai::mpc::quadruped {

TerrainReceiverSynchronizedModule::TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel> &terrainModel,
                                                                     ros::NodeHandle &nodeHandle)
    : terrainModelPtr_(&terrainModel),
      segmentedPlanesRos_(new tbai::mpc::quadruped::SegmentedPlanesTerrainModelRos(nodeHandle)) {}

void TerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                     const vector_t &currentState,
                                                     const ocs2::ReferenceManagerInterface &referenceManager) {
    if (auto newTerrain = segmentedPlanesRos_->getTerrainModel()) {
        terrainModelPtr_->reset(std::move(newTerrain));
        segmentedPlanesRos_->publish();
    }
}

}  // namespace tbai::mpc::quadruped
