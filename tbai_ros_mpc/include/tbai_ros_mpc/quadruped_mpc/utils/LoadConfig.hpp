#pragma once

#include <string>

#include <ocs2_core/Types.h>

namespace switched_model {

ocs2::vector_t getDefaultJointPosition(const std::string &configFile, const std::string &prefix);

}  // namespace switched_model
