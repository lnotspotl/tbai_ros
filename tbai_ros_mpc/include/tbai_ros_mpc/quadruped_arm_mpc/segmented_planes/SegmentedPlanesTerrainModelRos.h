//
// Created by rgrandia on 24.06.20.
// ARM version with 18-joint support
//

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <ocs2_core/misc/Benchmark.h>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include "tbai_ros_mpc/quadruped_arm_mpc/segmented_planes/SegmentedPlanesTerrainModel.h"

namespace tbai::mpc::quadruped_arm {

class SegmentedPlanesTerrainModelRos {
 public:
  SegmentedPlanesTerrainModelRos(ros::NodeHandle& nodehandle);

  ~SegmentedPlanesTerrainModelRos();

  /// Extract the latest terrain model. Resets internal model to a nullptr
  std::unique_ptr<SegmentedPlanesTerrainModel> getTerrainModel();

  void createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates, const Eigen::Vector3d& maxCoordinates);

  void publish();

 private:
  void callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> getSignedDistanceRange(const grid_map::GridMap& gridMap, const std::string& elevationLayer);

  ros::Subscriber terrainSubscriber_;
  ros::Publisher distanceFieldPublisher_;

  std::mutex updateMutex_;
  std::atomic_bool terrainUpdated_;
  std::unique_ptr<SegmentedPlanesTerrainModel> terrainPtr_;

  std::mutex updateCoordinatesMutex_;
  Eigen::Vector3d minCoordinates_;
  Eigen::Vector3d maxCoordinates_;
  bool externalCoordinatesGiven_;

  std::mutex pointCloudMutex_;
  std::unique_ptr<sensor_msgs::PointCloud2> pointCloud2MsgPtr_;

  ocs2::benchmark::RepeatedTimer callbackTimer_;
};

}  // namespace tbai::mpc::quadruped_arm
