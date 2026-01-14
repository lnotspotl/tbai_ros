#pragma once

#include <string>

#include <ocs2_core/Types.h>

namespace tbai::mpc::quadruped_arm {

ocs2::vector_t getDefaultJointPosition(const std::string &configFile, const std::string &prefix);

}  // namespace tbai::mpc::quadruped_arm
