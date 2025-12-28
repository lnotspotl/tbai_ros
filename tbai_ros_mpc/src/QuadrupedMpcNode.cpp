//
// Created by rgrandia on 17.02.20.
//

#include "tbai_ros_mpc/QuadrupedMpcNode.h"

#include <tbai_ros_ocs2/MPC_ROS_Interface.hpp>
#include <tbai_ros_ocs2/RosReferenceManager.hpp>

#include <tbai_ros_ocs2/logic/GaitReceiver.h>
#include <tbai_ros_ocs2/terrain/TerrainPlane.h>

#include <tbai_ros_ocs2/quadruped_interface/SwingPlanningVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainPlaneVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/LocalTerrainReceiver.h>

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr) {
  const std::string robotName = "anymal";

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

  // launch MPC nodes
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}
}  // namespace switched_model
