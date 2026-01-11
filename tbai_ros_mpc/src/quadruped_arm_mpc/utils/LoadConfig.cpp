#include "tbai_ros_mpc/quadruped_arm_mpc/utils/LoadConfig.hpp"

#include <ocs2_core/misc/LoadData.h>

namespace tbai::mpc::quadruped_arm {

ocs2::vector_t getDefaultJointPosition(const std::string &configFile, const std::string &prefix) {
    ocs2::vector_t defaultJointState;
    defaultJointState.setZero(12);
    ocs2::loadData::loadEigenMatrix(configFile, prefix + "defaultJointState", defaultJointState);
    return defaultJointState;
}

}  // namespace tbai::mpc::quadruped_arm
