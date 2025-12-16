#pragma once

#include <tbai_deploy_go2w/Go2WConstants.hpp>
#include <tbai_ros_static/StaticController.hpp>

namespace tbai {
namespace go2w {

class Go2WStaticController : public tbai::static_::RosStaticController {
   public:
    Go2WStaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr)
        : RosStaticController(stateSubscriberPtr) {}

    // Override to extract 16 joints from Go2W state vector
    vector_t jointAnglesFromState(const State &state) override {
        // State layout: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 16 joint pos
        return state.x.segment(12, GO2W_NUM_JOINTS);
    }

    std::string getName() const override { return "Go2WStaticController"; }
};

}  // namespace go2w
}  // namespace tbai
