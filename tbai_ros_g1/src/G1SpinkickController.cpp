#include <ros/ros.h>
#include <tbai_ros_g1/G1SpinkickController.hpp>

namespace tbai {
namespace g1 {

RosG1SpinkickController::RosG1SpinkickController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                                 const std::string &policyPath,
                                                 const std::string &controllerName, bool useModelMetaConfig,
                                                 float actionBeta)
    : G1SpinkickController(stateSubscriberPtr, policyPath, controllerName, useModelMetaConfig, actionBeta) {
}

void RosG1SpinkickController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1SpinkickController::preStep(currentTime, dt);
}

bool RosG1SpinkickController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
