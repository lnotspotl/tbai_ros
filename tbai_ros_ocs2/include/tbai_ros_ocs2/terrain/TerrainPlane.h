//
// Created by rgrandia on 21.04.20.
//

#pragma once

// Re-export TerrainPlane from tbai_mpc
#include <tbai_mpc/quadruped_mpc/terrain/TerrainPlane.h>

namespace tbai::mpc::quadruped {

// ROS-specific loading function
TerrainPlane loadTerrainPlane(const std::string& filename, bool verbose);

}  // namespace tbai::mpc::quadruped
