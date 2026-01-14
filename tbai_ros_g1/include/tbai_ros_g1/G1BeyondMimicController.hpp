#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1BeyondMimicController.hpp>

namespace tbai {
namespace g1 {

class RosG1BeyondMimicController : public tbai::g1::G1BeyondMimicController {
   public:
    RosG1BeyondMimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                const std::string &policyPath,
                                const std::string &controllerName = "RosG1BeyondMimicController",
                                bool useModelMetaConfig = true, float actionBeta = 1.0f);

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
