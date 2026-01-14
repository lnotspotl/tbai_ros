// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <mutex>

#include <geometry_msgs/Twist.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ros/init.h>
#include <ros/package.h>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_arm_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_arm_mpc/core/Rotations.h>
#include <tbai_mpc/quadruped_arm_mpc/quadruped_interfaces/Interfaces.h>
#include <tbai_ros_mpc/quadruped_arm_mpc/visualization/QuadrupedVisualizer.h>
#include <tbai_ros_ocs2/MRT_ROS_Interface.hpp>

using namespace ocs2;
using namespace tbai::mpc::quadruped_arm;

// Global variables for twist command
std::mutex twistMutex;
geometry_msgs::Twist latestTwist;
bool twistReceived = false;

void twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(twistMutex);
    latestTwist = *msg;
    twistReceived = true;
}

TargetTrajectories generateTargetTrajectories(const SystemObservation &observation, const geometry_msgs::Twist &twist,
                                              const vector_t &initialState, scalar_t horizon, scalar_t dt) {
    // Number of trajectory points
    const size_t N = static_cast<size_t>(horizon / dt) + 1;

    scalar_array_t timeTrajectory(N);
    vector_array_t stateTrajectory(N);
    vector_array_t inputTrajectory(N);

    // Extract current state
    const auto currentBasePose = getBasePose(observation.state);
    const auto currentOrientation = getOrientation(currentBasePose);
    const scalar_t currentYaw = currentOrientation.z();

    // Desired velocities in base frame
    const scalar_t vx_base = twist.linear.x;
    const scalar_t vy_base = twist.linear.y;
    const scalar_t wz = twist.angular.z;

    // Get neutral arm position from initial state (indices 24-29)
    const auto neutralArmPosition = initialState.segment<6>(24);

    for (size_t i = 0; i < N; ++i) {
        const scalar_t t = observation.time + i * dt;
        timeTrajectory[i] = t;

        // Start from initial state (includes neutral leg and arm positions)
        vector_t state = initialState;

        // Predict yaw
        const scalar_t predictedYaw = currentYaw + wz * (i * dt);

        // Rotate velocity from base to world frame
        const scalar_t cosYaw = std::cos(predictedYaw);
        const scalar_t sinYaw = std::sin(predictedYaw);
        const scalar_t vx_world = vx_base * cosYaw - vy_base * sinYaw;
        const scalar_t vy_world = vx_base * sinYaw + vy_base * cosYaw;

        // Update orientation
        state(0) = 0.0;           // roll
        state(1) = 0.0;           // pitch
        state(2) = predictedYaw;  // yaw

        // Update position prediction
        state(3) = observation.state(3) + vx_world * (i * dt);  // x
        state(4) = observation.state(4) + vy_world * (i * dt);  // y
        // z stays from initial state (standing height)

        // Set base velocities (in base frame)
        state(6) = 0.0;       // angular vel x
        state(7) = 0.0;       // angular vel y
        state(8) = wz;        // angular vel z
        state(9) = vx_base;   // linear vel x
        state(10) = vy_base;  // linear vel y
        state(11) = 0.0;      // linear vel z

        // Arm joints are already set from initialState (neutral position)
        // Explicitly ensure they stay at neutral
        state.segment<6>(24) = neutralArmPosition;

        stateTrajectory[i] = state;

        // Input (zero for now - MPC will compute optimal inputs)
        vector_t input = vector_t::Zero(INPUT_DIM);
        inputTrajectory[i] = input;
    }

    return TargetTrajectories(std::move(timeTrajectory), std::move(stateTrajectory), std::move(inputTrajectory));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "quadruped_mrt");
    ros::NodeHandle nodeHandle;

    // Get robot name from config
    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    ROS_INFO_STREAM("Starting dummy MRT for robot: " << robotName);

    // Get URDF string
    std::string urdfString;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/robot_description", urdfString),
                      "Failed to get parameter /robot_description");

    // Task settings
    std::string taskSettingsFile;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/task_settings_file", taskSettingsFile),
                      "Failed to get parameter /task_settings_file");

    // Frame declarations
    std::string frameDeclarationFile;
    TBAI_THROW_UNLESS(nodeHandle.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get parameter /frame_declaration_file");

    // Create quadruped interface based on robot type
    std::unique_ptr<tbai::mpc::quadruped_arm::QuadrupedInterface> quadrupedInterface;
    if (robotName == "anymal_d" || robotName == "anymal_c" || robotName == "anymal_b") {
        quadrupedInterface = tbai::mpc::quadruped_arm::getAnymalInterface(
            urdfString, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskSettingsFile),
            tbai::mpc::quadruped_arm::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName == "go2") {
        quadrupedInterface = tbai::mpc::quadruped_arm::getGo2Interface(
            urdfString, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskSettingsFile),
            tbai::mpc::quadruped_arm::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName == "spot" || robotName == "spot_arm") {
        quadrupedInterface = tbai::mpc::quadruped_arm::getSpotInterface(
            urdfString, tbai::mpc::quadruped_arm::loadQuadrupedSettings(taskSettingsFile),
            tbai::mpc::quadruped_arm::frameDeclarationFromFile(frameDeclarationFile));
    } else {
        TBAI_THROW("Robot name not supported: {}", robotName);
    }

    // Subscribe to twist commands
    std::string twistTopic = tbai::fromGlobalConfig<std::string>("reference_generator/twist/topic", "cmd_vel");
    ros::Subscriber twistSub = nodeHandle.subscribe(twistTopic, 1, twistCallback);
    ROS_INFO_STREAM("Subscribing to twist commands on: " << twistTopic);

    // MRT
    tbai::ocs2_ros::MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&quadrupedInterface->getRollout());
    mrt.launchNodes(nodeHandle);

    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    std::swap(jointNames[3], jointNames[6]);
    std::swap(jointNames[4], jointNames[7]);
    std::swap(jointNames[5], jointNames[8]);

    // Visualization
    auto baseName = tbai::fromGlobalConfig<std::string>("base_name");
    auto visualizer = std::make_shared<tbai::mpc::quadruped_arm::QuadrupedVisualizer>(
        quadrupedInterface->getKinematicModel(), jointNames, baseName, nodeHandle);

    // Initial observation
    SystemObservation observation;
    observation.state = quadrupedInterface->getInitialState();
    observation.input.setZero(INPUT_DIM);
    observation.mode = tbai::mpc::quadruped_arm::ModeNumber::STANCE;
    observation.time = 0.0;

    // Store initial state for reference (includes neutral arm position)
    const vector_t initialState = observation.state;

    // Initial target (use initial state as target)
    const TargetTrajectories initTargetTrajectories({observation.time}, {observation.state}, {observation.input});

    // Reset MPC node
    mrt.resetMpcNode(initTargetTrajectories);

    // Wait for the initial policy
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);
    while (!mrt.initialPolicyReceived() && ros::ok()) {
        mrt.spinMRT();
        mrt.setCurrentObservation(observation);
        ros::Duration(1.0 / mpcSettings.mrtDesiredFrequency_).sleep();
    }
    ROS_INFO("Initial policy received.");

    // Frequency settings
    const scalar_t mrtDesiredFrequency = mpcSettings.mrtDesiredFrequency_;
    const scalar_t dt = 1.0 / mrtDesiredFrequency;
    const scalar_t horizon = mpcSettings.timeHorizon_;
    const scalar_t targetDt = 0.1;  // Target trajectory discretization

    // Dummy loop
    ros::Rate rate(mrtDesiredFrequency);
    while (ros::ok()) {
        // Spin ROS callbacks (including twist subscriber)
        ros::spinOnce();

        // Spin MRT callbacks
        mrt.spinMRT();

        // Get latest twist command
        geometry_msgs::Twist currentTwist;
        {
            std::lock_guard<std::mutex> lock(twistMutex);
            currentTwist = latestTwist;
        }

        // Generate target trajectories from twist command (use initialState for neutral arm position)
        auto targetTrajectories =
            generateTargetTrajectories(observation, currentTwist, initialState, horizon, targetDt);

        // Update MPC target
        mrt.setTargetTrajectories(targetTrajectories);

        // Update policy if new one is available
        mrt.updatePolicy();

        // Evaluate policy at current time
        vector_t optimalState, optimalInput;
        size_t subsystem;
        mrt.evaluatePolicy(observation.time, observation.state, optimalState, optimalInput, subsystem);

        // Forward simulate
        SystemObservation nextObservation;
        nextObservation.time = observation.time + dt;
        nextObservation.input = optimalInput;
        nextObservation.mode = subsystem;

        // Use optimal state from MPC as next state for visualization
        nextObservation.state = optimalState;

        // Send observation to MPC
        mrt.setCurrentObservation(observation);

        // Update visualization
        visualizer->update(observation, mrt.getPolicy(), mrt.getCommand());

        // Update observation
        observation = nextObservation;

        rate.sleep();
    }

    return 0;
}
