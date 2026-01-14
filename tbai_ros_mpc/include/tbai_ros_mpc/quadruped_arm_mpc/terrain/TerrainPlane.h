//
// Created by rgrandia on 21.04.20.
// ARM version with 18-joint support
//

#pragma once

// Re-export TerrainPlane from tbai_mpc quadruped_arm_mpc
#include <tbai_mpc/quadruped_arm_mpc/terrain/TerrainPlane.h>

namespace tbai::mpc::quadruped_arm {

// ROS-specific loading function
TerrainPlane loadTerrainPlane(const std::string& filename, bool verbose);

}  // namespace tbai::mpc::quadruped_arm
