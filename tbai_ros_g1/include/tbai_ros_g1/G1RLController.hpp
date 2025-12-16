#pragma once

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <Eigen/Dense>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_g1/G1Constants.hpp>
#include <onnxruntime_cxx_api.h>

namespace tbai {
namespace g1 {

class G1RLController : public tbai::Controller {
   public:
    G1RLController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                   const std::string &policyPath);

    ~G1RLController();

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return "G1RLController"; }

    void stopController() override {}

    bool ok() const override;

    scalar_t getRate() const override { return 50.0; }  // 50Hz, matches step_dt=0.02

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

   private:
    void initOnnxRuntime(const std::string &policyPath);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();
    void updateHistory();
    vector3_t computeProjectedGravity(const quaternion_t &orientation) const;

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGenPtr_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
    std::vector<const char*> inputNames_;
    std::vector<const char*> outputNames_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    // Observation term histories (each term has its own history buffer)
    // This matches UNITREE's format: [term0_h0..h4, term1_h0..h4, ...]
    std::deque<vector3_t> angVelHistory_;       // 3 values * 5 history
    std::deque<vector3_t> projGravHistory_;     // 3 values * 5 history
    std::deque<vector3_t> velCmdHistory_;       // 3 values * 5 history
    std::deque<vector_t> jointPosRelHistory_;   // 29 values * 5 history
    std::deque<vector_t> jointVelRelHistory_;   // 29 values * 5 history
    std::deque<vector_t> lastActionHistory_;    // 29 values * 5 history
    vector_t fullObservation_;

    // Action output
    vector_t action_;

    // Configuration
    vector_t defaultJointPos_;
    vector_t actionScale_;
    vector_t actionOffset_;
    vector_t stiffness_;
    vector_t damping_;
    std::vector<int> jointIdsMap_;  // DFS to BFS mapping

    // Observation scales
    scalar_t angVelScale_ = 0.2;
    scalar_t jointVelScale_ = 0.05;

    // Joint names in DDS order
    std::vector<std::string> jointNames_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
