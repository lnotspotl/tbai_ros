#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_go2w/Go2WDreamWaQController.hpp>

namespace tbai {
namespace go2w {

class RosGo2WDreamWaQController : public tbai::go2w::Go2WDreamWaQController {
   public:
    RosGo2WDreamWaQController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                              const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                              const std::string &modelDir);

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace go2w
}  // namespace tbai
