//
// Created by rgrandia on 23.06.20.
//

#pragma once

#include <tbai_mpc/quadruped_mpc/terrain/TerrainModel.h>

#include <convex_plane_decomposition/PlanarRegion.h>

#include "tbai_ros_ocs2/segmented_planes/SegmentedPlanesSignedDistanceField.h"

namespace tbai::mpc::quadruped {

class SegmentedPlanesTerrainModel : public tbai::mpc::quadruped::TerrainModel {
 public:
  SegmentedPlanesTerrainModel(convex_plane_decomposition::PlanarTerrain planarTerrain);

  TerrainPlane getLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld,
                                                            std::function<scalar_t(const vector3_t&)> penaltyFunction) const override;

  using tbai::mpc::quadruped::TerrainModel::getConvexTerrainAtPositionInWorld;
  ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld,
                                                  std::function<scalar_t(const vector3_t&)> penaltyFunction) const override;

  void createSignedDistanceBetween(const Eigen::Vector3d& minCoordinates, const Eigen::Vector3d& maxCoordinates);

  const SegmentedPlanesSignedDistanceField* getSignedDistanceField() const override { return signedDistanceField_.get(); }

  vector3_t getHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  std::vector<vector2_t> getHeightProfileAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const override;

  const convex_plane_decomposition::PlanarTerrain& planarTerrain() const { return planarTerrain_; }

 private:
  const convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::unique_ptr<SegmentedPlanesSignedDistanceField> signedDistanceField_;
  const grid_map::Matrix* const elevationData_;
};

}  // namespace tbai::mpc::quadruped
