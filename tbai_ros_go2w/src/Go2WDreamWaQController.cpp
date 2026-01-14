#include <ros/ros.h>
#include <tbai_ros_go2w/Go2WDreamWaQController.hpp>

namespace tbai {
namespace go2w {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
RosGo2WDreamWaQController::RosGo2WDreamWaQController(
    const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr, const std::string &modelDir)
    : Go2WDreamWaQController(stateSubscriberPtr, refVelGenPtr, modelDir) {}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void RosGo2WDreamWaQController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    Go2WDreamWaQController::preStep(currentTime, dt);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool RosGo2WDreamWaQController::ok() const {
    return ros::ok();
}

}  // namespace go2w
}  // namespace tbai
