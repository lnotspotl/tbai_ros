#include "tbai_ros_np3o/Np3oController.hpp"

#include <tbai_core/config/Config.hpp>
#include <tbai_ros_msgs/EstimatedState.h>

namespace tbai {
namespace np3o {

RosNp3oController::RosNp3oController(const std::string &urdfString,
                                     const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::Np3oController(urdfString, stateSubscriberPtr, refVelGenPtr) {
    timeSinceLastVisualizationUpdate_ = 1000.0;

    publishState_ = tbai::fromGlobalConfig<bool>("np3o/publish_state", false);
    TBAI_LOG_INFO(logger_, "Controller state publishing is {}", publishState_ ? "enabled" : "disabled");

    if (publishState_) {
        statePublisher_ = ros::NodeHandle().advertise<tbai_ros_msgs::EstimatedState>(
            tbai::fromGlobalConfig<std::string>("np3o/state_topic", "estimated_state"), 10);
    }
}

void RosNp3oController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        ::tbai::np3o::State &state = np3oState_;  // post step, it's fine to use the already updated state
        stateVisualizer_.visualize(state);
        contactVisualizer_.visualize(state_.x, state_.contactFlags);
        if (publishState_) {
            publishEstimatedState();
        }
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
}

void RosNp3oController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);
}

void RosNp3oController::stopController() {
    // Implementation for stopController
}

bool RosNp3oController::ok() const {
    return ros::ok();
}

void RosNp3oController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = stateSubscriberPtr_->getLatestState();
}

void RosNp3oController::publishEstimatedState() {
    ::tbai::np3o::State &state = np3oState_;

    tbai_ros_msgs::EstimatedState stateMsg;
    stateMsg.timestamp = state_.timestamp;

    // Resize dynamic arrays
    stateMsg.joint_angles.resize(state.jointPositions.size());
    stateMsg.joint_velocities.resize(state.jointVelocities.size());
    stateMsg.contact_flags.resize(state_.contactFlags.size());

    std::copy(state.basePositionWorld.data(), state.basePositionWorld.data() + state.basePositionWorld.size(),
              stateMsg.base_position.begin());
    std::copy(state.baseOrientationWorld.data(), state.baseOrientationWorld.data() + state.baseOrientationWorld.size(),
              stateMsg.base_orientation_xyzw.begin());
    std::copy(state.baseLinearVelocityBase.data(),
              state.baseLinearVelocityBase.data() + state.baseLinearVelocityBase.size(), stateMsg.base_lin_vel.begin());
    std::copy(state.baseAngularVelocityBase.data(),
              state.baseAngularVelocityBase.data() + state.baseAngularVelocityBase.size(),
              stateMsg.base_ang_vel.begin());
    std::copy(state.jointPositions.data(), state.jointPositions.data() + state.jointPositions.size(),
              stateMsg.joint_angles.begin());
    std::copy(state.jointVelocities.data(), state.jointVelocities.data() + state.jointVelocities.size(),
              stateMsg.joint_velocities.begin());
    std::copy(state_.contactFlags.begin(), state_.contactFlags.end(), stateMsg.contact_flags.begin());

    statePublisher_.publish(stateMsg);
}

}  // namespace np3o
}  // namespace tbai
