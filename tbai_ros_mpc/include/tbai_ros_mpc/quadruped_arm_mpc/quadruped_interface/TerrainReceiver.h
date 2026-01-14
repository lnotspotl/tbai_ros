//
// Created by rgrandia on 28.09.20.
// ARM version with 18-joint support
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <tbai_mpc/quadruped_arm_mpc/terrain/TerrainModel.h>

#include <tbai_ros_mpc/quadruped_arm_mpc/segmented_planes/SegmentedPlanesTerrainModelRos.h>

namespace tbai::mpc::quadruped_arm {

class TerrainReceiverSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  TerrainReceiverSynchronizedModule(ocs2::Synchronized<TerrainModel>& terrainModel, ros::NodeHandle& nodeHandle);
  ~TerrainReceiverSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

 private:
  ocs2::Synchronized<TerrainModel>* terrainModelPtr_;
  std::unique_ptr<tbai::mpc::quadruped_arm::SegmentedPlanesTerrainModelRos> segmentedPlanesRos_;
};

}  // namespace tbai::mpc::quadruped_arm
