#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_deploy_go2w/Go2WConstants.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

namespace tbai {
namespace go2w {

using namespace torch::indexing;
using torch::jit::script::Module;

class Go2WDreamWaQController : public tbai::Controller {
   public:
    Go2WDreamWaQController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                           const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                           const std::string &modelDir);

    ~Go2WDreamWaQController() = default;

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return "Go2WDreamWaQ"; }

    void stopController() override {}

    bool ok() const override;

    scalar_t getRate() const override { return 50.0; }  // 50Hz control rate

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

   private:
    void loadModels(const std::string &modelDir);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void updateObservationHistory();
    at::Tensor runInference();
    at::Tensor reparameterize(const at::Tensor &mean, const at::Tensor &logvar);
    vector3_t computeProjectedGravity(const quaternion_t &orientation) const;

    // Transform between simulation and real joint ordering
    void transReal2Sim(vector_t &qj);
    void transSim2Real(vector_t &qj);

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGenPtr_;

    // TorchScript models
    Module actor_;
    Module encoder_;
    Module latentMu_;
    Module latentVar_;
    Module velMu_;
    Module velVar_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    // Observation history buffer (5 frames of 73 dimensions)
    std::deque<vector_t> obsHistory_;

    // Current observation (73 dimensions)
    vector_t currentObs_;

    // Configuration
    vector_t defaultSimAngles_;
    vector_t defaultRealAngles_;
    std::vector<scalar_t> kps_;
    std::vector<scalar_t> kds_;
    scalar_t actionScale_;

    // Observation scales
    scalar_t angVelScale_;
    scalar_t cmdScaleX_;
    scalar_t cmdScaleY_;
    scalar_t cmdScaleYaw_;
    scalar_t dofErrScale_;
    scalar_t dofVelScale_;

    // Wheel indices
    std::vector<int> wheelSimIndices_;
    std::vector<int> wheelRealIndices_;

    // Joint names in real robot order
    std::vector<std::string> jointNames_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace go2w
}  // namespace tbai
