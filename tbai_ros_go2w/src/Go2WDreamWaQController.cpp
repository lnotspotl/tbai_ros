#include <ros/ros.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_ros_go2w/Go2WDreamWaQController.hpp>

namespace tbai {
namespace go2w {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
static inline scalar_t clip(scalar_t x, scalar_t min, scalar_t max) {
    return std::max(min, std::min(x, max));
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
Go2WDreamWaQController::Go2WDreamWaQController(
    const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr, const std::string &modelDir)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGenPtr_(refVelGenPtr) {
    logger_ = tbai::getLogger("go2w_dreamwaq");

    TBAI_LOG_INFO(logger_, "Initializing Go2WDreamWaQController");
    TBAI_LOG_INFO(logger_, "Model directory: {}", modelDir);

    // Load configuration
    angVelScale_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/ang_vel_scale");
    cmdScaleX_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/cmd_scale_x");
    cmdScaleY_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/cmd_scale_y");
    cmdScaleYaw_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/cmd_scale_yaw");
    dofErrScale_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/dof_err_scale");
    dofVelScale_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/dof_vel_scale");
    actionScale_ = tbai::fromGlobalConfig<scalar_t>("go2w_controller/action_scale");

    kps_ = tbai::fromGlobalConfig<std::vector<scalar_t>>("go2w_controller/kps");
    kds_ = tbai::fromGlobalConfig<std::vector<scalar_t>>("go2w_controller/kds");

    auto defaultRealAnglesVec = tbai::fromGlobalConfig<std::vector<scalar_t>>("go2w_controller/default_real_angles");
    auto defaultSimAnglesVec = tbai::fromGlobalConfig<std::vector<scalar_t>>("go2w_controller/default_sim_angles");

    defaultRealAngles_ = Eigen::Map<vector_t>(defaultRealAnglesVec.data(), defaultRealAnglesVec.size());
    defaultSimAngles_ = Eigen::Map<vector_t>(defaultSimAnglesVec.data(), defaultSimAnglesVec.size());

    wheelSimIndices_ = tbai::fromGlobalConfig<std::vector<int>>("go2w_controller/wheel_sim_indices");
    wheelRealIndices_ = tbai::fromGlobalConfig<std::vector<int>>("go2w_controller/wheel_real_indices");

    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Load models
    loadModels(modelDir);

    // Initialize buffers
    lastAction_ = vector_t::Zero(GO2W_NUM_JOINTS);
    currentObs_ = vector_t::Zero(OBS_SIZE_PER_STEP);

    // Initialize observation history with zeros
    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        obsHistory_.push_back(vector_t::Zero(OBS_SIZE_PER_STEP));
    }

    TBAI_LOG_INFO(logger_, "Go2WDreamWaQController initialized");
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::loadModels(const std::string &modelDir) {
    try {
        std::string actorPath = modelDir + "/actor_dwaq.pt";
        std::string encoderPath = modelDir + "/encoder_dwaq.pt";
        std::string latentMuPath = modelDir + "/latent_mu_dwaq.pt";
        std::string latentVarPath = modelDir + "/latent_var_dwaq.pt";
        std::string velMuPath = modelDir + "/vel_mu_dwaq.pt";
        std::string velVarPath = modelDir + "/vel_var_dwaq.pt";

        TBAI_LOG_INFO(logger_, "Loading actor from: {}", actorPath);
        actor_ = torch::jit::load(actorPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "Loading encoder from: {}", encoderPath);
        encoder_ = torch::jit::load(encoderPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "Loading latent_mu from: {}", latentMuPath);
        latentMu_ = torch::jit::load(latentMuPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "Loading latent_var from: {}", latentVarPath);
        latentVar_ = torch::jit::load(latentVarPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "Loading vel_mu from: {}", velMuPath);
        velMu_ = torch::jit::load(velMuPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "Loading vel_var from: {}", velVarPath);
        velVar_ = torch::jit::load(velVarPath, torch::kCPU);

        TBAI_LOG_INFO(logger_, "All models loaded successfully");
    } catch (const c10::Error &e) {
        TBAI_THROW("Failed to load DreamWaQ models: {}", e.what());
    }
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();  // Process ROS callbacks (including /cmd_vel)
    state_ = stateSubscriberPtr_->getLatestState();
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool Go2WDreamWaQController::isSupported(const std::string &controllerType) {
    return (controllerType == "Go2WDreamWaQ" || controllerType == "DREAMWAQ");
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool Go2WDreamWaQController::ok() const {
    return true;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
bool Go2WDreamWaQController::checkStability() const {
    scalar_t roll = state_.x[0];
    if (roll >= 1.57 || roll <= -1.57) {
        return false;
    }
    scalar_t pitch = state_.x[1];
    if (pitch >= 1.57 || pitch <= -1.57) {
        return false;
    }
    return true;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::changeController(const std::string &controllerType, scalar_t currentTime) {
    // Reset observation history when controller changes
    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        obsHistory_[i].setZero();
    }
    lastAction_.setZero();
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
vector3_t Go2WDreamWaQController::computeProjectedGravity(const quaternion_t &orientation) const {
    // Rotate gravity vector from world to body frame
    matrix3_t R = orientation.toRotationMatrix();
    vector3_t gravityWorld(0, 0, -1);
    return R.transpose() * gravityWorld;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::transReal2Sim(vector_t &qj) {
    // Transform from real robot ordering to simulation ordering
    // Real: FR(0-2,wheel12), FL(3-5,wheel13), RR(6-8,wheel14), RL(9-11,wheel15)
    // Sim: FR_hip,FR_thigh,FR_calf,FR_wheel, FL_hip,FL_thigh,FL_calf,FL_wheel, ...
    vector_t tmp = qj;

    // FR leg (real 0-2, wheel 12) -> sim (0-3)
    tmp[0] = qj[3];  // FL_hip -> sim pos 0 (FR_hip in sim order is different)
    tmp[1] = qj[4];
    tmp[2] = qj[5];
    tmp[3] = qj[13];

    // FL leg (real 3-5, wheel 13) -> sim (4-7)
    tmp[4] = qj[0];
    tmp[5] = qj[1];
    tmp[6] = qj[2];
    tmp[7] = qj[12];

    // RR leg (real 6-8, wheel 14) -> sim (8-11)
    tmp[8] = qj[9];
    tmp[9] = qj[10];
    tmp[10] = qj[11];
    tmp[11] = qj[15];

    // RL leg (real 9-11, wheel 15) -> sim (12-15)
    tmp[12] = qj[6];
    tmp[13] = qj[7];
    tmp[14] = qj[8];
    tmp[15] = qj[14];

    qj = tmp;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::transSim2Real(vector_t &qj) {
    // Transform from simulation ordering to real robot ordering
    vector_t tmp = qj;

    tmp[3] = qj[0];
    tmp[4] = qj[1];
    tmp[5] = qj[2];
    tmp[13] = qj[3];

    tmp[0] = qj[4];
    tmp[1] = qj[5];
    tmp[2] = qj[6];
    tmp[12] = qj[7];

    tmp[9] = qj[8];
    tmp[10] = qj[9];
    tmp[11] = qj[10];
    tmp[15] = qj[11];

    tmp[6] = qj[12];
    tmp[7] = qj[13];
    tmp[8] = qj[14];
    tmp[14] = qj[15];

    qj = tmp;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Extract state components
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t angVel = state_.x.segment<3>(6);

    // Get joint positions and velocities (16 DOF)
    vector_t qj = state_.x.segment(12, GO2W_NUM_JOINTS);
    vector_t dqj = state_.x.segment(12 + GO2W_NUM_JOINTS, GO2W_NUM_JOINTS);

    // Transform to simulation ordering
    transReal2Sim(qj);
    transReal2Sim(dqj);

    // Also transform last action to sim ordering for observation
    vector_t lastActionSim = lastAction_;
    transReal2Sim(lastActionSim);

    // Compute projected gravity
    quaternion_t q = tbai::ocs2rpy2quat(rpy);
    vector3_t gravityOrientation = computeProjectedGravity(q);

    // Get velocity command
    auto cmd = refVelGenPtr_->getReferenceVelocity(currentTime, dt);

    // Build observation vector (73 dimensions)
    int idx = 0;

    // Angular velocity (3) - scaled
    currentObs_.segment<3>(idx) = angVel * angVelScale_;
    idx += 3;

    // Gravity orientation (3)
    currentObs_.segment<3>(idx) = gravityOrientation;
    idx += 3;

    // Command (3) - scaled
    currentObs_[idx++] = cmd.velocity_x * cmdScaleX_;
    currentObs_[idx++] = cmd.velocity_y * cmdScaleY_;
    currentObs_[idx++] = cmd.yaw_rate * cmdScaleYaw_;

    // Joint position error (16) - zero out wheel positions
    vector_t errObs = qj - defaultSimAngles_;
    for (int wheelIdx : wheelSimIndices_) {
        errObs[wheelIdx] = 0;
    }
    currentObs_.segment(idx, GO2W_NUM_JOINTS) = errObs * dofErrScale_;
    idx += GO2W_NUM_JOINTS;

    // Joint velocities (16) - scaled
    currentObs_.segment(idx, GO2W_NUM_JOINTS) = dqj * dofVelScale_;
    idx += GO2W_NUM_JOINTS;

    // Joint positions (16) - zero out wheel positions
    vector_t qjObs = qj;
    for (int wheelIdx : wheelSimIndices_) {
        qjObs[wheelIdx] = 0;
    }
    currentObs_.segment(idx, GO2W_NUM_JOINTS) = qjObs;
    idx += GO2W_NUM_JOINTS;

    // Last action (16)
    currentObs_.segment(idx, GO2W_NUM_JOINTS) = lastActionSim;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void Go2WDreamWaQController::updateObservationHistory() {
    // Shift history buffer and add new observation
    obsHistory_.pop_front();
    obsHistory_.push_back(currentObs_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
at::Tensor Go2WDreamWaQController::reparameterize(const at::Tensor &mean, const at::Tensor &logvar) {
    at::Tensor var = torch::exp(logvar * 0.5);
    at::Tensor noise = torch::randn_like(var);
    return mean + var * noise;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
at::Tensor Go2WDreamWaQController::runInference() {
    torch::NoGradGuard no_grad;

    // Build observation history tensor (365 = 73 * 5)
    std::vector<float> obsHistVec;
    obsHistVec.reserve(TOTAL_OBS_HISTORY_SIZE);

    for (const auto &obs : obsHistory_) {
        for (int i = 0; i < obs.size(); ++i) {
            obsHistVec.push_back(static_cast<float>(obs[i]));
        }
    }

    at::Tensor obsHistTensor =
        torch::from_blob(obsHistVec.data(), {TOTAL_OBS_HISTORY_SIZE}, torch::TensorOptions().dtype(torch::kFloat32))
            .clone();

    // Run encoder
    at::Tensor h = encoder_.forward({obsHistTensor}).toTensor();

    // Get latent distribution parameters
    at::Tensor velMu = velMu_.forward({h}).toTensor();
    at::Tensor velVar = velVar_.forward({h}).toTensor();
    at::Tensor latentMu = latentMu_.forward({h}).toTensor();
    at::Tensor latentVar = latentVar_.forward({h}).toTensor();

    // Reparameterize
    at::Tensor vel = reparameterize(velMu, velVar);
    at::Tensor latent = reparameterize(latentMu, latentVar);

    // Concatenate encoder output
    at::Tensor code = torch::cat({vel, latent}, -1);

    // Build current observation tensor
    std::vector<float> currentObsVec(currentObs_.data(), currentObs_.data() + currentObs_.size());
    at::Tensor currentObsTensor =
        torch::from_blob(currentObsVec.data(), {OBS_SIZE_PER_STEP}, torch::TensorOptions().dtype(torch::kFloat32))
            .clone();

    // Concatenate code with current observation for actor input
    at::Tensor obsAll = torch::cat({code, currentObsTensor}, -1);

    // Run actor
    at::Tensor action = actor_.forward({obsAll}).toTensor();

    return action;
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
std::vector<tbai::MotorCommand> Go2WDreamWaQController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    auto t1 = std::chrono::high_resolution_clock::now();

    // Build current observation
    buildObservation(currentTime, dt);

    // Update observation history BEFORE inference (matches Python implementation)
    // This ensures the encoder sees the current observation in the history
    updateObservationHistory();

    auto t2 = std::chrono::high_resolution_clock::now();

    // Run neural network inference
    at::Tensor actionTensor = runInference();

    auto t3 = std::chrono::high_resolution_clock::now();

    // Convert action tensor to Eigen vector
    vector_t actionSim(GO2W_NUM_JOINTS);
    auto actionPtr = actionTensor.data_ptr<float>();
    for (int i = 0; i < GO2W_NUM_JOINTS; ++i) {
        actionSim[i] = static_cast<scalar_t>(actionPtr[i]);
    }

    // Transform action to real robot ordering
    vector_t actionReal = actionSim;
    transSim2Real(actionReal);

    // Store action for next step (in real ordering)
    lastAction_ = actionReal;

    // Generate motor commands
    std::vector<tbai::MotorCommand> motorCommands;
    motorCommands.resize(GO2W_NUM_JOINTS);

    for (int i = 0; i < GO2W_NUM_JOINTS; ++i) {
        tbai::MotorCommand &cmd = motorCommands[i];
        cmd.joint_name = jointNames_[i];
        cmd.kp = kps_[i];
        cmd.kd = kds_[i];
        cmd.torque_ff = 0.0;

        // Check if this is a wheel joint (velocity control)
        bool isWheel = false;
        for (int wheelIdx : wheelRealIndices_) {
            if (i == wheelIdx) {
                isWheel = true;
                break;
            }
        }

        if (isWheel) {
            // Wheel: velocity control
            cmd.desired_position = 0.0;
            cmd.desired_velocity = actionReal[i] * 10.0;  // Wheel speed scale
        } else {
            // Leg joint: position control
            cmd.desired_position = defaultRealAngles_[i] + actionReal[i] * actionScale_;
            cmd.desired_velocity = 0.0;
        }
    }

    auto t4 = std::chrono::high_resolution_clock::now();

    TBAI_LOG_INFO_THROTTLE(logger_, 10.0, "Obs build: {} ms, NN inference: {} ms, Total: {} ms",
                           std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0,
                           std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0,
                           std::chrono::duration_cast<std::chrono::microseconds>(t4 - t1).count() / 1000.0);

    return motorCommands;
}

}  // namespace go2w
}  // namespace tbai
