#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1MimicController.hpp>

namespace tbai {
namespace g1 {

class RosG1MimicController : public tbai::g1::G1MimicController {
   public:
    RosG1MimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                         const std::string &policyPath, const std::string &motionFilePath, float motionFps = 60.0f,
                         float timeStart = 0.0f, float timeEnd = -1.0f,
                         const std::string &controllerName = "RosG1MimicController");

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
