#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_ros_g1/G1Constants.hpp>
#include <tbai_ros_g1/MotionLoader.hpp>

namespace tbai {
namespace g1 {

// Mimic observation dimensions (no history, history_length=1)
constexpr int MIMIC_OBS_MOTION_CMD = 58;  // 29 joint pos + 29 joint vel from motion
constexpr int MIMIC_OBS_ANCHOR_ORI = 6;   // 6D rotation matrix representation
constexpr int MIMIC_OBS_BASE_ANG_VEL = 3;
constexpr int MIMIC_OBS_JOINT_POS_REL = 29;
constexpr int MIMIC_OBS_JOINT_VEL_REL = 29;
constexpr int MIMIC_OBS_LAST_ACTION = 29;
constexpr int MIMIC_TOTAL_OBS_SIZE = MIMIC_OBS_MOTION_CMD + MIMIC_OBS_ANCHOR_ORI + MIMIC_OBS_BASE_ANG_VEL +
                                     MIMIC_OBS_JOINT_POS_REL + MIMIC_OBS_JOINT_VEL_REL + MIMIC_OBS_LAST_ACTION;  // 154

class G1MimicController : public tbai::Controller {
   public:
    G1MimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr, const std::string &policyPath,
                      const std::string &motionFilePath, float motionFps = 60.0f, float timeStart = 0.0f,
                      float timeEnd = -1.0f,  // -1 means use full duration
                      const std::string &controllerName = "G1MimicController");

    ~G1MimicController();

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override;

    std::vector<tbai::MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override {}

    bool isSupported(const std::string &controllerType) override;

    std::string getName() const override { return controllerName_; }

    void stopController() override;

    bool ok() const override;

    scalar_t getRate() const override { return 50.0; }  // 50Hz, matches step_dt=0.02

    bool checkStability() const override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    // Check if motion playback has completed
    bool isMotionComplete() const;

    // Get current motion time
    float getMotionTime() const { return currentMotionTime_; }

   private:
    void initOnnxRuntime(const std::string &policyPath);
    void buildObservation(scalar_t currentTime, scalar_t dt);
    void runInference();

    // Compute torso orientation from root + waist motors
    quaternion_t computeTorsoOrientation(const quaternion_t &rootQuat, const vector_t &jointPos) const;

    // Compute 6D rotation representation
    Eigen::Matrix<scalar_t, 6, 1> computeOrientationError(const quaternion_t &targetQuat,
                                                          const quaternion_t &actualQuat) const;

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;
    std::unique_ptr<MotionLoader> motionLoader_;

    // ONNX Runtime
    std::unique_ptr<Ort::Env> ortEnv_;
    std::unique_ptr<Ort::Session> ortSession_;
    std::unique_ptr<Ort::MemoryInfo> memoryInfo_;
    std::vector<const char *> inputNames_;
    std::vector<const char *> outputNames_;

    // State
    tbai::State state_;
    vector_t lastAction_;

    // Observation (no history for mimic controller)
    vector_t observation_;

    // Action output
    vector_t action_;

    // Configuration
    vector_t defaultJointPos_;
    vector_t actionScale_;
    vector_t actionOffset_;
    vector_t stiffness_;
    vector_t damping_;
    std::vector<int> jointIdsMap_;  // DFS to BFS mapping

    // Joint names in DDS order
    std::vector<std::string> jointNames_;

    // Motion playback timing
    float timeStart_;
    float timeEnd_;
    float currentMotionTime_;
    bool motionActive_;

    // Initial orientation alignment (robot yaw to motion yaw)
    quaternion_t initQuat_;
    matrix3_t worldToInit_;

    // Controller name (for distinguishing multiple mimic controllers)
    std::string controllerName_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace g1
}  // namespace tbai
