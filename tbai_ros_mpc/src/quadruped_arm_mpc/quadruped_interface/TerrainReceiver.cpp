//
// Created by rgrandia on 28.09.20.
// ARM version with 18-joint support
//

#include "tbai_ros_mpc/quadruped_arm_mpc/quadruped_interface/TerrainReceiver.h"

namespace tbai::mpc::quadruped_arm {

TerrainReceiverSynchronizedModule::TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel> &terrainModel,
                                                                     ros::NodeHandle &nodeHandle)
    : terrainModelPtr_(&terrainModel),
      segmentedPlanesRos_(new tbai::mpc::quadruped_arm::SegmentedPlanesTerrainModelRos(nodeHandle)) {}

void TerrainReceiverSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime,
                                                     const vector_t &currentState,
                                                     const ocs2::ReferenceManagerInterface &referenceManager) {
    if (auto newTerrain = segmentedPlanesRos_->getTerrainModel()) {
        terrainModelPtr_->reset(std::move(newTerrain));
        segmentedPlanesRos_->publish();
    }
}

}  // namespace tbai::mpc::quadruped_arm
