#include "tbai_ros_g1/G1RLController.hpp"

#include <algorithm>
#include <ros/ros.h>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_reference/ReferenceVelocity.hpp>

namespace tbai {
namespace g1 {

G1RLController::G1RLController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                               const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                               const std::string &policyPath)
    : stateSubscriberPtr_(stateSubscriberPtr), refVelGenPtr_(refVelGenPtr) {
    logger_ = tbai::getLogger("G1RLController");
    TBAI_LOG_INFO(logger_, "Initializing G1RLController");

    // Initialize vectors
    lastAction_ = vector_t::Zero(G1_NUM_JOINTS);
    action_ = vector_t::Zero(G1_NUM_JOINTS);
    fullObservation_ = vector_t::Zero(TOTAL_OBS_SIZE);

    // Load configuration
    defaultJointPos_ = vector_t(G1_NUM_JOINTS);
    actionScale_ = vector_t(G1_NUM_JOINTS);
    actionOffset_ = vector_t(G1_NUM_JOINTS);
    stiffness_ = vector_t(G1_NUM_JOINTS);
    damping_ = vector_t(G1_NUM_JOINTS);

    // Load from config
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/default_joint_pos");
    auto actionScaleVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/action_scale");
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_controller/damping");
    jointIdsMap_ = tbai::fromGlobalConfig<std::vector<int>>("g1_controller/joint_ids_map");

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
        actionScale_[i] = actionScaleVec[i];
        actionOffset_[i] = defaultJointPosVec[i];  // Offset is same as default
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

    angVelScale_ = tbai::fromGlobalConfig<double>("g1_controller/ang_vel_scale", 0.2);
    jointVelScale_ = tbai::fromGlobalConfig<double>("g1_controller/joint_vel_scale", 0.05);

    // Initialize joint names (in DDS order)
    jointNames_ = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
    };

    // Initialize observation term histories with zeros
    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        angVelHistory_.push_back(vector3_t::Zero());
        projGravHistory_.push_back(vector3_t::Zero());
        velCmdHistory_.push_back(vector3_t::Zero());
        jointPosRelHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
        jointVelRelHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
        lastActionHistory_.push_back(vector_t::Zero(G1_NUM_JOINTS));
    }

    // Initialize ONNX runtime
    initOnnxRuntime(policyPath);

    TBAI_LOG_INFO(logger_, "G1RLController initialized successfully");
}

G1RLController::~G1RLController() {
    TBAI_LOG_INFO(logger_, "Destroying G1RLController");
}

void G1RLController::initOnnxRuntime(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1RLController");

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, policyPath.c_str(), sessionOptions);
    memoryInfo_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Get input/output names
    Ort::AllocatorWithDefaultOptions allocator;

    auto inputNamePtr = ortSession_->GetInputNameAllocated(0, allocator);
    inputNames_.push_back(strdup(inputNamePtr.get()));

    auto outputNamePtr = ortSession_->GetOutputNameAllocated(0, allocator);
    outputNames_.push_back(strdup(outputNamePtr.get()));

    TBAI_LOG_INFO(logger_, "ONNX model loaded. Input: {}, Output: {}", inputNames_[0], outputNames_[0]);
}

void G1RLController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = stateSubscriberPtr_->getLatestState();
}

vector3_t G1RLController::computeProjectedGravity(const quaternion_t &orientation) const {
    // Project gravity vector [0, 0, -1] into body frame
    vector3_t gravityWorld(0.0, 0.0, -1.0);
    matrix3_t R = orientation.toRotationMatrix();
    return R.transpose() * gravityWorld;
}

void G1RLController::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Get state components
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Compute quaternion from RPY for projected gravity
    quaternion_t orientation = tbai::rpy2quat(rpy);

    // Get velocity command from reference generator
    auto refVel = refVelGenPtr_->getReferenceVelocity(currentTime, dt);
    // Clamp velocity commands to policy-expected ranges
    // From deploy.yaml: lin_vel_x: [-0.5, 1.0], lin_vel_y: [-0.3, 0.3], ang_vel_z: [-0.2, 0.2]
    vector3_t velCmd;
    velCmd[0] = std::clamp(refVel.velocity_x, -0.5, 1.0);
    velCmd[1] = std::clamp(refVel.velocity_y, -0.3, 0.3);
    velCmd[2] = std::clamp(refVel.yaw_rate, -0.5, 0.5);

    // Compute each observation term (scaled as per deploy.yaml)
    // Term 0: base_ang_vel * scale [0.2, 0.2, 0.2]
    vector3_t angVelScaled = baseAngVel * angVelScale_;

    // Term 1: projected_gravity * scale [1.0, 1.0, 1.0]
    vector3_t projGrav = computeProjectedGravity(orientation);

    // Term 2: velocity_commands * scale [1.0, 1.0, 1.0]
    // (already clamped above)

    // Term 3: joint_pos_rel * scale [1.0, ...] (using joint_ids_map for reordering)
    vector_t jointPosRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointPosRel[i] = jointPos[mappedIdx] - defaultJointPos_[i];
    }

    // Term 4: joint_vel_rel * scale [0.05, ...] (using joint_ids_map for reordering)
    vector_t jointVelRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointVelRel[i] = jointVel[mappedIdx] * jointVelScale_;
    }

    // Term 5: last_action * scale [1.0, ...]
    // (lastAction_ is already the raw action from previous step)

    // Add to history buffers (push back, pop front)
    angVelHistory_.push_back(angVelScaled);
    angVelHistory_.pop_front();

    projGravHistory_.push_back(projGrav);
    projGravHistory_.pop_front();

    velCmdHistory_.push_back(velCmd);
    velCmdHistory_.pop_front();

    jointPosRelHistory_.push_back(jointPosRel);
    jointPosRelHistory_.pop_front();

    jointVelRelHistory_.push_back(jointVelRel);
    jointVelRelHistory_.pop_front();

    lastActionHistory_.push_back(lastAction_);
    lastActionHistory_.pop_front();
}

void G1RLController::updateHistory() {
    int idx = 0;

    // Term 0: base_ang_vel (3 * 5 = 15)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = angVelHistory_[h];
        idx += 3;
    }

    // Term 1: projected_gravity (3 * 5 = 15)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = projGravHistory_[h];
        idx += 3;
    }

    // Term 2: velocity_commands (3 * 5 = 15)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment<3>(idx) = velCmdHistory_[h];
        idx += 3;
    }

    // Term 3: joint_pos_rel (29 * 5 = 145)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = jointPosRelHistory_[h];
        idx += G1_NUM_JOINTS;
    }

    // Term 4: joint_vel_rel (29 * 5 = 145)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = jointVelRelHistory_[h];
        idx += G1_NUM_JOINTS;
    }

    // Term 5: last_action (29 * 5 = 145)
    for (int h = 0; h < HISTORY_LENGTH; ++h) {
        fullObservation_.segment(idx, G1_NUM_JOINTS) = lastActionHistory_[h];
        idx += G1_NUM_JOINTS;
    }
    // Total: 15 + 15 + 15 + 145 + 145 + 145 = 480
}

void G1RLController::runInference() {
    // Prepare input tensor
    std::vector<int64_t> inputShape = {1, TOTAL_OBS_SIZE};
    std::vector<float> inputData(TOTAL_OBS_SIZE);
    for (int i = 0; i < TOTAL_OBS_SIZE; ++i) {
        inputData[i] = static_cast<float>(fullObservation_[i]);
    }

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        *memoryInfo_, inputData.data(), inputData.size(), inputShape.data(), inputShape.size());

    // Run inference
    auto outputTensors = ortSession_->Run(
        Ort::RunOptions{nullptr}, inputNames_.data(), &inputTensor, 1, outputNames_.data(), 1);

    // Get output
    float *outputData = outputTensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        action_[i] = static_cast<scalar_t>(outputData[i]);
    }

    // Store for next step
    lastAction_ = action_;
}

std::vector<tbai::MotorCommand> G1RLController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    updateHistory();
    runInference();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;

        // Map from policy order back to DDS order
        int ddsIdx = jointIdsMap_[i];
        cmd.joint_name = jointNames_[ddsIdx];

        // Apply action scaling and offset (scale and offset are in policy order)
        cmd.desired_position = actionScale_[i] * action_[i] + actionOffset_[i];
        cmd.desired_velocity = 0.0;
        // Stiffness and damping are stored in DDS order (matching the motor indexing)
        cmd.kp = stiffness_[ddsIdx];
        cmd.kd = damping_[ddsIdx];
        cmd.torque_ff = 0.0;

        commands.push_back(cmd);
    }

    return commands;
}

bool G1RLController::isSupported(const std::string &controllerType) {
    return controllerType == "G1RLController" || controllerType == "g1_rl" || controllerType == "g1";
}

bool G1RLController::ok() const {
    return ros::ok();
}

bool G1RLController::checkStability() const {
    const scalar_t maxAngle = 1.0;
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

void G1RLController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to G1RLController");

    // Reset action history
    lastAction_.setZero();

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    // Compute current observation terms
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);
    quaternion_t orientation = tbai::rpy2quat(rpy);

    vector3_t angVelScaled = baseAngVel * angVelScale_;
    vector3_t projGrav = computeProjectedGravity(orientation);
    vector3_t velCmd = vector3_t::Zero();  // Start with zero velocity command

    vector_t jointPosRel(G1_NUM_JOINTS);
    vector_t jointVelRel(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        jointPosRel[i] = jointPos[mappedIdx] - defaultJointPos_[i];
        jointVelRel[i] = jointVel[mappedIdx] * jointVelScale_;
    }

    angVelHistory_.clear();
    projGravHistory_.clear();
    velCmdHistory_.clear();
    jointPosRelHistory_.clear();
    jointVelRelHistory_.clear();
    lastActionHistory_.clear();

    for (int i = 0; i < HISTORY_LENGTH; ++i) {
        angVelHistory_.push_back(angVelScaled);
        projGravHistory_.push_back(projGrav);
        velCmdHistory_.push_back(velCmd);
        jointPosRelHistory_.push_back(jointPosRel);
        jointVelRelHistory_.push_back(jointVelRel);
        lastActionHistory_.push_back(lastAction_);  // zeros
    }

    TBAI_LOG_INFO(logger_, "Observation history initialized with current state");
}

}  // namespace g1
}  // namespace tbai
