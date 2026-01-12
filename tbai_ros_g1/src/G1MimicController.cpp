#include <ros/ros.h>
#include <tbai_ros_g1/G1MimicController.hpp>

namespace tbai {
namespace g1 {

RosG1MimicController::RosG1MimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                           const std::string &policyPath, const std::string &motionFilePath,
                                           float motionFps, float timeStart, float timeEnd,
                                           const std::string &controllerName)
    : G1MimicController(stateSubscriberPtr, policyPath, motionFilePath, motionFps, timeStart, timeEnd, controllerName) {
}

void RosG1MimicController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1MimicController::preStep(currentTime, dt);
}

bool RosG1MimicController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
