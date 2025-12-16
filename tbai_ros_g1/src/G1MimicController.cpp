#include "tbai_ros_g1/G1MimicController.hpp"

#include <algorithm>
#include <ros/ros.h>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/Rotations.hpp>

namespace tbai {
namespace g1 {

G1MimicController::G1MimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                     const std::string &policyPath,
                                     const std::string &motionFilePath,
                                     float motionFps,
                                     float timeStart,
                                     float timeEnd,
                                     const std::string &controllerName)
    : stateSubscriberPtr_(stateSubscriberPtr),
      timeStart_(timeStart),
      currentMotionTime_(0.0f),
      motionActive_(false),
      controllerName_(controllerName) {
    logger_ = tbai::getLogger(controllerName_);
    TBAI_LOG_INFO(logger_, "Initializing {}", controllerName_);

    // Load motion data
    motionLoader_ = std::make_unique<MotionLoader>(motionFilePath, motionFps);
    TBAI_LOG_INFO(logger_, "Loaded motion file with {} frames, duration: {:.2f}s",
                  motionLoader_->numFrames(), motionLoader_->duration());

    // Set time end (use full duration if -1)
    timeEnd_ = (timeEnd < 0) ? motionLoader_->duration() : timeEnd;
    timeEnd_ = std::clamp(timeEnd_, 0.0f, motionLoader_->duration());
    timeStart_ = std::clamp(timeStart_, 0.0f, timeEnd_);

    TBAI_LOG_INFO(logger_, "Motion playback range: {:.2f}s - {:.2f}s", timeStart_, timeEnd_);

    // Initialize vectors
    lastAction_ = vector_t::Zero(G1_NUM_JOINTS);
    action_ = vector_t::Zero(G1_NUM_JOINTS);
    observation_ = vector_t::Zero(MIMIC_TOTAL_OBS_SIZE);

    // Load mimic-specific configuration
    defaultJointPos_ = vector_t(G1_NUM_JOINTS);
    actionScale_ = vector_t(G1_NUM_JOINTS);
    actionOffset_ = vector_t(G1_NUM_JOINTS);
    stiffness_ = vector_t(G1_NUM_JOINTS);
    damping_ = vector_t(G1_NUM_JOINTS);

    // Load from config (mimic-specific parameters)
    auto defaultJointPosVec = tbai::fromGlobalConfig<std::vector<double>>("g1_mimic_controller/default_joint_pos");
    auto actionScaleVec = tbai::fromGlobalConfig<std::vector<double>>("g1_mimic_controller/action_scale");
    auto stiffnessVec = tbai::fromGlobalConfig<std::vector<double>>("g1_mimic_controller/stiffness");
    auto dampingVec = tbai::fromGlobalConfig<std::vector<double>>("g1_mimic_controller/damping");
    jointIdsMap_ = tbai::fromGlobalConfig<std::vector<int>>("g1_mimic_controller/joint_ids_map");

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        defaultJointPos_[i] = defaultJointPosVec[i];
        actionScale_[i] = actionScaleVec[i];
        actionOffset_[i] = defaultJointPosVec[i];  // Offset is same as default
        stiffness_[i] = stiffnessVec[i];
        damping_[i] = dampingVec[i];
    }

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

    initOnnxRuntime(policyPath);
    initQuat_ = quaternion_t::Identity();
    worldToInit_.setIdentity();

    TBAI_LOG_INFO(logger_, "{} initialized successfully", controllerName_);
}

G1MimicController::~G1MimicController() {
    TBAI_LOG_INFO(logger_, "Destroying {}", controllerName_);
}

void G1MimicController::initOnnxRuntime(const std::string &policyPath) {
    TBAI_LOG_INFO(logger_, "Loading ONNX policy from: {}", policyPath);

    ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "G1MimicController");

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

void G1MimicController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = stateSubscriberPtr_->getLatestState();

    // Update motion time
    if (motionActive_) {
        currentMotionTime_ += static_cast<float>(dt);
        float clampedTime = std::clamp(currentMotionTime_ + timeStart_, timeStart_, timeEnd_);
        motionLoader_->update(clampedTime);
    }
}

quaternion_t G1MimicController::computeTorsoOrientation(const quaternion_t &rootQuat,
                                                         const vector_t &jointPos) const {
    // Torso orientation includes waist motors (indices 12, 13, 14 in DDS order)
    // waist_yaw (12), waist_roll (13), waist_pitch (14)
    quaternion_t torsoQuat = rootQuat
        * quaternion_t(Eigen::AngleAxis<scalar_t>(jointPos[12], vector3_t::UnitZ()))
        * quaternion_t(Eigen::AngleAxis<scalar_t>(jointPos[13], vector3_t::UnitX()))
        * quaternion_t(Eigen::AngleAxis<scalar_t>(jointPos[14], vector3_t::UnitY()));

    return torsoQuat;
}

Eigen::Matrix<scalar_t, 6, 1> G1MimicController::computeOrientationError(
    const quaternion_t &targetQuat,
    const quaternion_t &actualQuat) const {
    // Compute relative rotation: (initQuat * targetQuat)^-1 * actualQuat
    quaternion_t relQuat = (initQuat_ * targetQuat).conjugate() * actualQuat;
    matrix3_t rot = relQuat.toRotationMatrix().transpose();

    // Extract 6D representation (first two columns)
    Eigen::Matrix<scalar_t, 6, 1> data;
    data << rot(0, 0), rot(0, 1), rot(1, 0), rot(1, 1), rot(2, 0), rot(2, 1);
    return data;
}

void G1MimicController::buildObservation(scalar_t currentTime, scalar_t dt) {
    // Get state components
    vector3_t rpy = state_.x.segment<3>(0);
    vector3_t baseAngVel = state_.x.segment<3>(6);
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    vector_t jointVel = state_.x.segment(12 + G1_NUM_JOINTS, G1_NUM_JOINTS);

    // Compute root quaternion from RPY
    quaternion_t rootQuat = tbai::rpy2quat(rpy);

    // Get motion target data (already in DFS order)
    vector_t motionJointPosDfs = motionLoader_->jointPos();
    vector_t motionJointVelDfs = motionLoader_->jointVel();

    // Convert motion data from DFS to BFS order using joint_ids_map
    vector_t motionJointPosBfs(G1_NUM_JOINTS);
    vector_t motionJointVelBfs(G1_NUM_JOINTS);
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        motionJointPosBfs[i] = motionJointPosDfs[jointIdsMap_[i]];
        motionJointVelBfs[i] = motionJointVelDfs[jointIdsMap_[i]];
    }

    // Compute torso orientations
    quaternion_t robotTorsoQuat = computeTorsoOrientation(rootQuat, jointPos);
    quaternion_t motionTorsoQuat = computeTorsoOrientation(
        motionLoader_->rootQuaternion(), motionJointPosDfs);

    // Build observation vector (154 dimensions total)
    int idx = 0;

    // 1. motion_command: target joint positions + velocities (58 dims)
    observation_.segment(idx, G1_NUM_JOINTS) = motionJointPosBfs;
    idx += G1_NUM_JOINTS;
    observation_.segment(idx, G1_NUM_JOINTS) = motionJointVelBfs;
    idx += G1_NUM_JOINTS;

    // 2. motion_anchor_ori_b: 6D orientation error (6 dims)
    auto oriError = computeOrientationError(motionTorsoQuat, robotTorsoQuat);
    observation_.segment<6>(idx) = oriError;
    idx += 6;

    // 3. base_ang_vel (3 dims)
    observation_.segment<3>(idx) = baseAngVel;
    idx += 3;

    // 4. joint_pos_rel: current joint positions relative to default (29 dims)
    // Reorder using joint_ids_map (DDS -> policy order)
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        observation_[idx + i] = jointPos[mappedIdx] - defaultJointPos_[i];
    }
    idx += G1_NUM_JOINTS;

    // 5. joint_vel_rel: current joint velocities (29 dims)
    // Reorder using joint_ids_map
    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        int mappedIdx = jointIdsMap_[i];
        observation_[idx + i] = jointVel[mappedIdx];
    }
    idx += G1_NUM_JOINTS;

    // 6. last_action (29 dims)
    observation_.segment(idx, G1_NUM_JOINTS) = lastAction_;
}

void G1MimicController::runInference() {
    // Prepare input tensor
    std::vector<int64_t> inputShape = {1, MIMIC_TOTAL_OBS_SIZE};
    std::vector<float> inputData(MIMIC_TOTAL_OBS_SIZE);
    for (int i = 0; i < MIMIC_TOTAL_OBS_SIZE; ++i) {
        inputData[i] = static_cast<float>(observation_[i]);
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

std::vector<tbai::MotorCommand> G1MimicController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    buildObservation(currentTime, dt);
    runInference();

    std::vector<tbai::MotorCommand> commands;
    commands.reserve(G1_NUM_JOINTS);

    for (int i = 0; i < G1_NUM_JOINTS; ++i) {
        tbai::MotorCommand cmd;

        // Map from policy order back to DDS order
        int ddsIdx = jointIdsMap_[i];
        cmd.joint_name = jointNames_[ddsIdx];

        // Apply action scaling and offset
        cmd.desired_position = actionScale_[i] * action_[i] + actionOffset_[i];
        cmd.desired_velocity = 0.0;
        // Stiffness and damping are stored in DDS order
        cmd.kp = stiffness_[ddsIdx];
        cmd.kd = damping_[ddsIdx];
        cmd.torque_ff = 0.0;

        commands.push_back(cmd);
    }

    return commands;
}

bool G1MimicController::isSupported(const std::string &controllerType) {
    // Support the controller's specific name and generic mimic names
    return controllerType == controllerName_ ||
           controllerType == "G1MimicController" ||
           controllerType == "g1_mimic" ||
           controllerType == "mimic";
}

void G1MimicController::stopController() {
    motionActive_ = false;
    TBAI_LOG_INFO(logger_, "{} stopped", controllerName_);
}

bool G1MimicController::ok() const {
    return ros::ok();
}

bool G1MimicController::checkStability() const {
    // Check for excessive roll/pitch (same as walking controller)
    const scalar_t maxAngle = 1.0;  // ~57 degrees
    scalar_t roll = std::abs(state_.x[0]);
    scalar_t pitch = std::abs(state_.x[1]);

    if (roll > maxAngle || pitch > maxAngle) {
        TBAI_LOG_WARN(logger_, "Instability detected: roll={:.2f}, pitch={:.2f}", roll, pitch);
        return false;
    }
    return true;
}

bool G1MimicController::isMotionComplete() const {
    return currentMotionTime_ >= (timeEnd_ - timeStart_);
}

void G1MimicController::changeController(const std::string &controllerType, scalar_t currentTime) {
    TBAI_LOG_INFO(logger_, "Changing to {}", controllerName_);

    // Reset action
    lastAction_.setZero();

    // Get current state
    state_ = stateSubscriberPtr_->getLatestState();

    // Compute robot root orientation
    vector3_t rpy = state_.x.segment<3>(0);
    quaternion_t robotRootQuat = tbai::rpy2quat(rpy);

    // Get robot torso orientation
    vector_t jointPos = state_.x.segment(12, G1_NUM_JOINTS);
    quaternion_t robotTorsoQuat = computeTorsoOrientation(robotRootQuat, jointPos);

    // Reset motion loader with alignment
    motionLoader_->reset(robotRootQuat, timeStart_);

    // Get motion torso orientation at start
    vector_t motionJointPos = motionLoader_->jointPos();
    quaternion_t motionTorsoQuat = computeTorsoOrientation(
        motionLoader_->rootQuaternion(), motionJointPos);

    // Compute initial yaw alignment between robot and motion
    matrix3_t refYaw = MotionLoader::yawQuaternion(motionTorsoQuat).toRotationMatrix();
    matrix3_t robotYaw = MotionLoader::yawQuaternion(robotTorsoQuat).toRotationMatrix();
    initQuat_ = quaternion_t(robotYaw * refYaw.transpose());

    // Reset motion time
    currentMotionTime_ = 0.0f;
    motionActive_ = true;

    TBAI_LOG_INFO(logger_, "Motion playback started from {:.2f}s", timeStart_);
}

}  // namespace g1
}  // namespace tbai
