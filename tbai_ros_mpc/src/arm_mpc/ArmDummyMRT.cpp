#include <ocs2_mpc/SystemObservation.h>
#include <ros/init.h>
#include <ros/package.h>
#include <tbai_mpc/arm_mpc/ArmInterface.h>
#include <tbai_ros_mpc/arm_mpc/ArmDummyVisualization.h>
#include <tbai_ros_ocs2/MRT_ROS_Interface.hpp>

using namespace ocs2;
using namespace tbai::mpc::arm;

int main(int argc, char **argv) {
    const std::string robotName = "arm";

    ros::init(argc, argv, robotName + "_mrt");
    ros::NodeHandle nodeHandle;

    std::string taskFile, libFolder, urdfFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/libFolder", libFolder);
    nodeHandle.getParam("/urdfFile", urdfFile);
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;

    tbai::mpc::arm::ArmInterface interface(taskFile, libFolder, urdfFile);

    // MRT
    tbai::ocs2_ros::MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    auto dummyVisualization = std::make_shared<tbai::mpc::arm::ArmDummyVisualization>(nodeHandle, interface);

    // Initial observation
    SystemObservation observation;
    observation.state = interface.getInitialState();
    observation.input.setZero(interface.getArmModelInfo().inputDim);
    observation.time = 0.0;

    // Initial target
    vector_t initTarget(7);
    initTarget.head(3) << 0.5, 0, 0.5;
    initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    const vector_t zeroInput = vector_t::Zero(interface.getArmModelInfo().inputDim);
    const TargetTrajectories initTargetTrajectories({observation.time}, {initTarget}, {zeroInput});

    // Reset MPC node
    mrt.resetMpcNode(initTargetTrajectories);

    // Wait for the initial policy
    while (!mrt.initialPolicyReceived() && ros::ok()) {
        mrt.spinMRT();
        mrt.setCurrentObservation(observation);
        ros::Rate(interface.mpcSettings().mrtDesiredFrequency_).sleep();
    }
    ROS_INFO("Initial policy received.");

    // Frequency settings
    const scalar_t mrtDesiredFrequency = interface.mpcSettings().mrtDesiredFrequency_;
    const scalar_t mpcDesiredFrequency = interface.mpcSettings().mpcDesiredFrequency_;
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

        // Simple forward integration (Euler step with rollout dynamics)
        nextObservation.state = optimalState;  // Use optimal state from MPC as next state for visualization

        // Send observation to MPC
        mrt.setCurrentObservation(observation);

        // Update visualization
        dummyVisualization->update(observation, mrt.getPolicy(), mrt.getCommand());

        // Update observation
        observation = nextObservation;

        rate.sleep();
    }

    return 0;
}
