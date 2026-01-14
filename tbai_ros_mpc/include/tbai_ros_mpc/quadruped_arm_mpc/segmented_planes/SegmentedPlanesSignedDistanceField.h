//
// Created by rgrandia on 17.03.22.
// ARM version with 18-joint support
//

#pragma once

#include <tbai_mpc/quadruped_arm_mpc/terrain/SignedDistanceField.h>

#include <grid_map_sdf/SignedDistanceField.hpp>

namespace tbai::mpc::quadruped_arm {

/**
 * Simple wrapper class to implement the tbai::mpc::quadruped_arm::SignedDistanceField interface.
 * See the forwarded function for documentation.
 */
class SegmentedPlanesSignedDistanceField : public SignedDistanceField {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SegmentedPlanesSignedDistanceField(const grid_map::GridMap& gridMap, const std::string& elevationLayer, double minHeight,
                                     double maxHeight)
      : sdf_(gridMap, elevationLayer, minHeight, maxHeight) {}

  ~SegmentedPlanesSignedDistanceField() override = default;
  SegmentedPlanesSignedDistanceField* clone() const override { return new SegmentedPlanesSignedDistanceField(*this); };

  tbai::mpc::quadruped_arm::scalar_t value(const tbai::mpc::quadruped_arm::vector3_t& position) const override { return sdf_.value(position); }

  tbai::mpc::quadruped_arm::vector3_t derivative(const tbai::mpc::quadruped_arm::vector3_t& position) const override { return sdf_.derivative(position); }

  std::pair<tbai::mpc::quadruped_arm::scalar_t, tbai::mpc::quadruped_arm::vector3_t> valueAndDerivative(
      const tbai::mpc::quadruped_arm::vector3_t& position) const override {
    return sdf_.valueAndDerivative(position);
  }

  grid_map::SignedDistanceField& asGridmapSdf() { return sdf_; }
  const grid_map::SignedDistanceField& asGridmapSdf() const { return sdf_; }

 protected:
  SegmentedPlanesSignedDistanceField(const SegmentedPlanesSignedDistanceField& other) : sdf_(other.sdf_){};

 private:
  grid_map::SignedDistanceField sdf_;
};

}  // namespace tbai::mpc::quadruped_arm
