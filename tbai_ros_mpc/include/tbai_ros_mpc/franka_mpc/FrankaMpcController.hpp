#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_mpc/franka_mpc/FrankaInterface.h>
#include <tbai_mpc/franka_wbc/WbcBase.hpp>
#include <tbai_ros_mpc/franka_mpc/FrankaVisualizer.h>
#include <tbai_ros_mpc/franka_mpc/InteractiveMarkerTarget.h>

namespace tbai {
namespace mpc {
namespace franka {

class FrankaMpcController : public tbai::Controller {
   public:
    FrankaMpcController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                        std::function<scalar_t()> getCurrentTimeFunction);

    ~FrankaMpcController() override;

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    std::string getName() const override { return "FrankaMpcController"; }

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override { stopReferenceThread(); }

    scalar_t getRate() const override { return 400.0; }

    bool checkStability() const override { return isStable_; }

    bool ok() const override { return ros::ok(); }

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override;

   private:
    void initialize();
    void resetMpc();
    void setObservation();
    ocs2::SystemObservation generateSystemObservation() const;
    ocs2::TargetTrajectories generateReferenceTrajectory(scalar_t currentTime,
                                                         const ocs2::SystemObservation &observation);

    // Reference thread management
    void referenceThreadLoop();
    void startReferenceThread();
    void stopReferenceThread();

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::shared_ptr<spdlog::logger> logger_;

    std::unique_ptr<ocs2::franka::FrankaInterface> manipulatorInterfacePtr_;
    std::unique_ptr<WbcBase> wbcPtr_;
    std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpcPtr_;
    std::unique_ptr<ocs2::MPC_MRT_Interface> mrtPtr_;

    scalar_t initTime_ = 0.0;
    scalar_t tNow_ = 0.0;

    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 30.0;  // Same as quadruped MPC
    scalar_t timeSinceLastMpcUpdate_ = 1e5;

    bool isStable_ = true;

    State state_;

    std::function<scalar_t()> getCurrentTimeFunction_;

    // Reference thread
    std::atomic<bool> stopReferenceThread_{false};
    std::thread referenceThread_;
    scalar_t referenceThreadRate_ = 5.0;  // Same rate as quadruped MPC

    // Target end-effector pose (from reference or external input)
    vector_t targetEEPosition_;
    vector_t targetEEOrientation_;

    // Visualization
    std::unique_ptr<FrankaVisualizer> visualizerPtr_;
    scalar_t timeSinceLastVisualizationUpdate_ = 1e5;

    // Pinocchio interface for FK computation (needs mutable data)
    std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
    size_t eeFrameId_;

    // Current EE pose (computed in postStep)
    vector_t currentEEPosition_;
    vector_t currentEEOrientation_;

    const bool wbcOnly_ = true;

    // Helper to compute current EE pose
    void computeCurrentEEPose(const vector_t &jointPositions);

    // Compute EE position for given joint positions
    vector_t computeEEPosition(const vector_t &jointPositions);

    // Compute EE trajectory from MPC solution
    std::vector<vector_t> computeEETrajectory(const ocs2::PrimalSolution &primalSolution);

    // Cached EE trajectory for visualization
    std::vector<vector_t> eeTrajectory_;

    // Interactive marker target for RViz control
    std::unique_ptr<InteractiveMarkerTarget> interactiveMarkerTargetPtr_;
};

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
