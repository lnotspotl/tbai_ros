#include <ros/ros.h>
#include <tbai_ros_g1/G1RLController.hpp>

namespace tbai {
namespace g1 {

RosG1RLController::RosG1RLController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                                     const std::string &policyPath)
    : G1RLController(stateSubscriberPtr, refVelGenPtr, policyPath) {}

void RosG1RLController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1RLController::preStep(currentTime, dt);
}

bool RosG1RLController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
