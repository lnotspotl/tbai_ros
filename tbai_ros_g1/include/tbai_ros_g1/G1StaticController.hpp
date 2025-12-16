#pragma once

#include <tbai_ros_static/StaticController.hpp>
#include <tbai_ros_g1/G1Constants.hpp>

namespace tbai {
namespace g1 {

class G1StaticController : public tbai::static_::RosStaticController {
   public:
    G1StaticController(std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr)
        : RosStaticController(stateSubscriberPtr) {}

    vector_t jointAnglesFromState(const State &state) override {
        // State layout: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 29 joint pos
        return state.x.segment(12, G1_NUM_JOINTS);
    }

    std::string getName() const override { return "G1StaticController"; }
};

}  // namespace g1
}  // namespace tbai
