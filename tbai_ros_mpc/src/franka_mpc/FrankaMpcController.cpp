// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_mpc/franka_mpc/FrankaMpcController.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ros/package.h>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_mpc/franka_wbc/Factory.hpp>

namespace tbai {
namespace mpc {
namespace franka {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
FrankaMpcController::FrankaMpcController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                         std::function<scalar_t()> getCurrentTimeFunction)
    : stateSubscriberPtr_(stateSubscriberPtr), getCurrentTimeFunction_(getCurrentTimeFunction) {
    logger_ = tbai::getLogger("franka_mpc_controller");
    initTime_ = tbai::readInitTime();
    initialize();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
FrankaMpcController::~FrankaMpcController() {
    stopReferenceThread();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::initialize() {
    // Load configuration paths from ROS parameters (set by launch file)
    ros::NodeHandle nh;
    std::string taskFile, libFolder, urdfFile;
    nh.getParam("/taskFile", taskFile);
    nh.getParam("/libFolder", libFolder);
    nh.getParam("/urdfFile", urdfFile);

    std::cerr << "[FrankaMpcController] Loading task file: " << taskFile << std::endl;
    std::cerr << "[FrankaMpcController] Loading library folder: " << libFolder << std::endl;
    std::cerr << "[FrankaMpcController] Loading URDF file: " << urdfFile << std::endl;

    // Create manipulator interface
    manipulatorInterfacePtr_ =
        std::make_unique<ocs2::franka::FrankaInterface>(taskFile, libFolder, urdfFile);

    // Get URDF string for WBC
    std::ifstream urdfStream(urdfFile);
    std::string urdfString((std::istreambuf_iterator<char>(urdfStream)), std::istreambuf_iterator<char>());

    // Create WBC
    wbcPtr_ = getWbcUnique(taskFile, urdfString, manipulatorInterfacePtr_->getFrankaModelInfo());

    // Create MPC
    mpcPtr_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
        manipulatorInterfacePtr_->mpcSettings(), manipulatorInterfacePtr_->ddpSettings(),
        manipulatorInterfacePtr_->getRollout(), manipulatorInterfacePtr_->getOptimalControlProblem(),
        manipulatorInterfacePtr_->getInitializer());
    mpcPtr_->getSolverPtr()->setReferenceManager(manipulatorInterfacePtr_->getReferenceManagerPtr());

    // Create MRT interface (takes reference to MPC) - same as quadruped
    mrtPtr_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpcPtr_, true);  // threaded = true

    // Initialize target EE pose
    targetEEPosition_ = vector_t::Zero(3);
    targetEEPosition_(2) = 0.5;
    targetEEPosition_(0) = 0.5;
    targetEEOrientation_ = vector_t::Zero(4);
    targetEEOrientation_(1) = 1.0;  // Identity quaternion (x=0, y=0, z=0, w=1)

    // Initialize current EE pose
    currentEEPosition_ = vector_t::Zero(3);
    currentEEOrientation_ = vector_t::Zero(4);
    currentEEOrientation_(3) = 1.0;

    // Create Pinocchio interface copy for EE pose computation (needs mutable data for FK)
    pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(
        manipulatorInterfacePtr_->getPinocchioInterface());
    eeFrameId_ = pinocchioInterfacePtr_->getModel().getFrameId(
        manipulatorInterfacePtr_->getFrankaModelInfo().eeFrame);

    // Create visualizer with joint names
    const auto& jointNames = manipulatorInterfacePtr_->getFrankaModelInfo().dofNames;
    visualizerPtr_ = std::make_unique<FrankaVisualizer>(nh, jointNames);

    // Create interactive marker target for RViz control
    Eigen::Vector3d initialPosition(targetEEPosition_(0), targetEEPosition_(1), targetEEPosition_(2));
    Eigen::Quaterniond initialOrientation(targetEEOrientation_(3), targetEEOrientation_(0),
                                           targetEEOrientation_(1), targetEEOrientation_(2));  // w, x, y, z
    interactiveMarkerTargetPtr_ = std::make_unique<InteractiveMarkerTarget>(
        nh, "panda_link0", initialPosition, initialOrientation);

    tNow_ = 0.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<MotorCommand> FrankaMpcController::getMotorCommands(scalar_t currentTime, scalar_t dt) {
    // Same pattern as quadruped MpcController
    mrtPtr_->spinMRT();
    mrtPtr_->updatePolicy();

    tNow_ = getCurrentTimeFunction_() - initTime_;

    auto observation = generateSystemObservation();

    // Read target from interactive marker (thread-safe)
    interactiveMarkerTargetPtr_->getTargetPose(targetEEPosition_, targetEEOrientation_);

    constexpr bool debug = false;

    ocs2::vector_t desiredState;
    ocs2::vector_t desiredInput;
    ocs2::vector_t desiredJointAcceleration;
    
    desiredState = vector_t::Zero(7);
    desiredInput = vector_t::Zero(7);
    desiredJointAcceleration = vector_t::Zero(7);
    
    // Evaluate MPC policy
    if (!debug) {
        size_t desiredMode;
        mrtPtr_->evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);

        // Compute desired joint accelerations using finite differences (same as quadruped)
        constexpr ocs2::scalar_t time_eps = 1e-3;
        ocs2::vector_t dummyState;
        ocs2::vector_t dummyInput;
        size_t dummyMode;
        mrtPtr_->evaluatePolicy(tNow_ + time_eps, observation.state, dummyState, dummyInput, dummyMode);

        const size_t nJoints = 7;
        desiredJointAcceleration = 0 * (dummyInput.tail(nJoints) - desiredInput.tail(nJoints)) / time_eps;
    } else {
        desiredState = observation.state;
        desiredInput = observation.input;
        desiredJointAcceleration = vector_t::Zero(7);
    }

    // Get motor commands from WBC
    auto commands = wbcPtr_->getMotorCommands(tNow_, observation.state, observation.input, desiredState, desiredInput,
                                               desiredJointAcceleration, targetEEPosition_, targetEEOrientation_,
                                               isStable_);

    // Update MPC observation and target periodically (same as quadruped)
    timeSinceLastMpcUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_) {
        setObservation();

        // Update target trajectory (inline, no separate thread)
        vector_t targetState(7);
        targetState.head(3) = targetEEPosition_;
        targetState.tail(4) = targetEEOrientation_;
        const vector_t zeroInput = vector_t::Zero(manipulatorInterfacePtr_->getFrankaModelInfo().inputDim);
        const scalar_t horizon = 2.0;
        ocs2::TargetTrajectories targetTrajectories(
            {observation.time, observation.time + horizon},
            {targetState, targetState},
            {zeroInput, zeroInput});
        mrtPtr_->setTargetTrajectories(targetTrajectories);
    }

    return commands;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = stateSubscriberPtr_->getLatestState();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::postStep(scalar_t currentTime, scalar_t dt) {
    timeSinceLastVisualizationUpdate_ += dt;
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 15.0) {
        // Get joint positions from state
        const auto& rbdState = state_.x;
        const size_t nJoints = manipulatorInterfacePtr_->getFrankaModelInfo().armDim;
        const vector_t jointPositions = rbdState.head(nJoints);

        // Compute current EE pose from joint positions
        computeCurrentEEPose(jointPositions);

        // Compute EE trajectory from MPC solution (make a copy for thread safety)
        ocs2::PrimalSolution primalSolution = mrtPtr_->getPolicy();
        eeTrajectory_ = computeEETrajectory(primalSolution);

        // Update visualizer (publishes robot state TF and markers)
        auto observation = generateSystemObservation();
        visualizerPtr_->update(jointPositions,
                               currentEEPosition_, currentEEOrientation_,
                               targetEEPosition_, targetEEOrientation_,
                               eeTrajectory_,
                               observation, primalSolution);

        timeSinceLastVisualizationUpdate_ = 0.0;
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::computeCurrentEEPose(const vector_t& jointPositions) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();

    // Forward kinematics
    pinocchio::forwardKinematics(model, data, jointPositions);
    pinocchio::updateFramePlacements(model, data);

    // Get EE pose
    const auto& eePlacement = data.oMf[eeFrameId_];

    // Extract position
    currentEEPosition_ = eePlacement.translation();

    // Extract orientation as quaternion (x, y, z, w)
    Eigen::Quaterniond quat(eePlacement.rotation());
    currentEEOrientation_(0) = quat.x();
    currentEEOrientation_(1) = quat.y();
    currentEEOrientation_(2) = quat.z();
    currentEEOrientation_(3) = quat.w();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
vector_t FrankaMpcController::computeEEPosition(const vector_t& jointPositions) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();

    pinocchio::forwardKinematics(model, data, jointPositions);
    pinocchio::updateFramePlacements(model, data);

    return data.oMf[eeFrameId_].translation();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
std::vector<vector_t> FrankaMpcController::computeEETrajectory(const ocs2::PrimalSolution& primalSolution) {
    std::vector<vector_t> eeTrajectory;
    eeTrajectory.reserve(primalSolution.stateTrajectory_.size());

    for (const auto& state : primalSolution.stateTrajectory_) {
        eeTrajectory.push_back(computeEEPosition(state));
    }

    return eeTrajectory;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::referenceThreadLoop() {
    // Same pattern as quadruped MpcController::referenceThreadLoop()
    TBAI_LOG_WARN(logger_, "Reference trajectory generator reset");

    // Wait for initial observation
    stateSubscriberPtr_->waitTillInitialized();
    ocs2::SystemObservation observation = generateSystemObservation();
    TBAI_LOG_WARN(logger_, "Reference trajectory generator initialized");

    // Reference loop - same structure as quadruped
    auto sleepDuration = std::chrono::milliseconds(static_cast<int>(1000.0 / referenceThreadRate_));
    while (!stopReferenceThread_) {
        // Build 7D target state: [position(3), quaternion(4)]
        // Quaternion order from Eigen::Quaternion::coeffs() is (x, y, z, w)
        vector_t targetState(7);
        targetState.head(3) = targetEEPosition_;
        targetState.tail(4) = targetEEOrientation_;

        auto observation = generateSystemObservation();

        // Zero input (joint velocities)
        const vector_t zeroInput = vector_t::Zero(manipulatorInterfacePtr_->getFrankaModelInfo().inputDim);

        // Create trajectory with current time and a future horizon point
        // This tells MPC to reach target and hold it
        const scalar_t horizon = 2.0;  // 2 seconds into the future
        ocs2::TargetTrajectories targetTrajectories(
            {observation.time, observation.time + horizon},
            {targetState, targetState},
            {zeroInput, zeroInput});
        mrtPtr_->setTargetTrajectories(targetTrajectories);

        TBAI_LOG_INFO_THROTTLE(logger_, 5.0, "Publishing reference");
        std::this_thread::sleep_for(sleepDuration);
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::TargetTrajectories FrankaMpcController::generateReferenceTrajectory(scalar_t currentTime,
                                                                           const ocs2::SystemObservation &observation) {
    // Simple hold-position reference trajectory
    // Similar to what a ReferenceTrajectoryGenerator would produce, but simpler for fixed-base arm
    const size_t trajKnots = 20;
    const scalar_t trajdt = 0.1;

    ocs2::scalar_array_t desiredTimeTrajectory(trajKnots);
    ocs2::vector_array_t desiredStateTrajectory(trajKnots);
    ocs2::vector_array_t desiredInputTrajectory(trajKnots);

    for (size_t i = 0; i < trajKnots; ++i) {
        desiredTimeTrajectory[i] = observation.time + i * trajdt;
        desiredStateTrajectory[i] = observation.state;
        desiredInputTrajectory[i] = observation.input;
    }

    return ocs2::TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                                    std::move(desiredInputTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::startReferenceThread() {
    // Same as quadruped
    stopReferenceThread();

    TBAI_LOG_WARN(logger_, "Starting reference thread");
    stopReferenceThread_ = false;
    referenceThread_ = std::thread(&FrankaMpcController::referenceThreadLoop, this);
    TBAI_LOG_WARN(logger_, "Reference thread started");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::stopReferenceThread() {
    stopReferenceThread_ = true;
    if (referenceThread_.joinable()) {
        referenceThread_.join();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::changeController(const std::string &controllerType, scalar_t currentTime) {
    // Same pattern as quadruped MpcController::changeController()
    preStep(currentTime, 0.0);

    if (!mrt_initialized_ || currentTime + 0.1 > mrtPtr_->getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = currentTime - initTime_;

    // Note: Reference updates are handled inline in getMotorCommands() at mpcRate_
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool FrankaMpcController::isSupported(const std::string &controllerType) {
    return controllerType == "WBC" || controllerType == "MPC";
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::resetMpc() {
    // Same pattern as quadruped MpcController::resetMpc()
    stateSubscriberPtr_->waitTillInitialized();
    auto initialObservation = generateSystemObservation();

    // Build initial EE target state: [position(3), quaternion(4)]
    vector_t initTarget(7);
    initTarget.head(3) = targetEEPosition_;
    initTarget.tail(4) = targetEEOrientation_;
    const vector_t zeroInput = vector_t::Zero(manipulatorInterfacePtr_->getFrankaModelInfo().inputDim);
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initTarget}, {zeroInput});
    mrtPtr_->resetMpcNode(initTargetTrajectories);

    while (!mrtPtr_->initialPolicyReceived()) {
        TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
        initialObservation = generateSystemObservation();
        mrtPtr_->setCurrentObservation(initialObservation);
        mrtPtr_->spinMRT();
        mrtPtr_->updatePolicy();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    TBAI_LOG_INFO(logger_, "Initial policy received.");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void FrankaMpcController::setObservation() {
    mrtPtr_->setCurrentObservation(generateSystemObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::SystemObservation FrankaMpcController::generateSystemObservation() const {
    auto state = stateSubscriberPtr_->getLatestState();
    const tbai::vector_t &rbdState = state.x;

    const size_t nJoints = manipulatorInterfacePtr_->getFrankaModelInfo().armDim;

    ocs2::SystemObservation observation;
    observation.time = state.timestamp - initTime_;
    observation.mode = 0;  // No mode switching for fixed-base manipulator

    // State = joint positions
    observation.state = rbdState.head(nJoints);

    // Input = joint velocities
    observation.input = rbdState.segment(nJoints, nJoints);

    return observation;
}

}  // namespace franka
}  // namespace mpc
}  // namespace tbai
