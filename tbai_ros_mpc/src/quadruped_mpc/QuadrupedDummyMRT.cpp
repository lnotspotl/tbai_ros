// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ros/init.h>
#include <ros/package.h>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/core/MotionPhaseDefinition.h>
#include <tbai_mpc/quadruped_mpc/quadruped_interfaces/Interfaces.h>
#include <tbai_ros_mpc/quadruped_mpc/visualization/QuadrupedVisualizer.h>
#include <tbai_ros_ocs2/MRT_ROS_Interface.hpp>

using namespace ocs2;
using namespace tbai::mpc::quadruped;

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
    std::unique_ptr<tbai::mpc::quadruped::QuadrupedInterface> quadrupedInterface;
    if (robotName == "anymal_d" || robotName == "anymal_c" || robotName == "anymal_b") {
        quadrupedInterface =
            tbai::mpc::quadruped::getAnymalInterface(urdfString, tbai::mpc::quadruped::loadQuadrupedSettings(taskSettingsFile),
                                       tbai::mpc::quadruped::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName == "go2") {
        quadrupedInterface =
            tbai::mpc::quadruped::getGo2Interface(urdfString, tbai::mpc::quadruped::loadQuadrupedSettings(taskSettingsFile),
                                    tbai::mpc::quadruped::frameDeclarationFromFile(frameDeclarationFile));
    } else if (robotName == "spot" || robotName == "spot_arm") {
        quadrupedInterface =
            tbai::mpc::quadruped::getSpotInterface(urdfString, tbai::mpc::quadruped::loadQuadrupedSettings(taskSettingsFile),
                                     tbai::mpc::quadruped::frameDeclarationFromFile(frameDeclarationFile));
    } else {
        TBAI_THROW("Robot name not supported: {}", robotName);
    }

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
    auto visualizer = std::make_shared<tbai::mpc::quadruped::QuadrupedVisualizer>(
        quadrupedInterface->getKinematicModel(), jointNames, baseName, nodeHandle);

    // Initial observation
    SystemObservation observation;
    observation.state = quadrupedInterface->getInitialState();
    observation.input.setZero(INPUT_DIM);
    observation.mode = tbai::mpc::quadruped::ModeNumber::STANCE;
    observation.time = 0.0;

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

    // Dummy loop
    ros::Rate rate(mrtDesiredFrequency);
    while (ros::ok()) {
        // Spin MRT callbacks
        mrt.spinMRT();

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
