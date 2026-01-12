#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1RLController.hpp>

namespace tbai {
namespace g1 {

class RosG1RLController : public tbai::g1::G1RLController {
   public:
    RosG1RLController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                      const std::string &policyPath);

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
