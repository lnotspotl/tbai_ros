#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ros/init.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>
#include <tbai_mpc/quadruped_mpc/quadruped_interfaces/Interfaces.h>
#include <tbai_ros_ocs2/MPC_ROS_Interface.hpp>
#include <tbai_ros_ocs2/RosReferenceManager.hpp>
#include <tbai_ros_ocs2/logic/GaitReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/LocalTerrainReceiver.h>
#include <tbai_ros_ocs2/quadruped_interface/SwingPlanningVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainPlaneVisualizer.h>
#include <tbai_ros_ocs2/quadruped_interface/TerrainReceiver.h>
#include <tbai_ros_ocs2/terrain/TerrainPlane.h>

namespace tbai::mpc::quadruped {

void quadrupedMpcNode(const std::string &robotName, ros::NodeHandle &nodeHandle,
                      const QuadrupedInterface &quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr) {
    auto solverModules = quadrupedInterface.getSynchronizedModules();

    // Gait
    auto gaitReceiver = std::make_shared<GaitReceiver>(
        nodeHandle, quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
    solverModules.push_back(gaitReceiver);

    // Terrain Receiver
    auto terrainReceiver = std::make_shared<TerrainReceiverSynchronizedModule>(
        quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nodeHandle);
    solverModules.push_back(terrainReceiver);

    // Local Terrain Receiver
    auto localTerrainReceiver = std::make_shared<LocalTerrainReceiverSynchronizedModule>(
        quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nodeHandle);
    solverModules.push_back(localTerrainReceiver);

    // Terrain plane visualization
    auto terrainVisualizer = std::make_shared<TerrainPlaneVisualizerSynchronizedModule>(
        quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
    solverModules.push_back(terrainVisualizer);

    // Swing planner
    auto swingPlanningVisualizer = std::make_shared<SwingPlanningVisualizer>(
        quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
    solverModules.push_back(swingPlanningVisualizer);

    // reference manager
    auto rosReferenceManagerPtr =
        std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);
    mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

    // MPC
    mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);

    // launch MPC nodes
    ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
    mpcNode.launchNodes(nodeHandle);
}

}  // namespace tbai::mpc::quadruped

using namespace tbai::mpc::quadruped;

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "quadruped_mpc_node");
    ros::NodeHandle nodeHandle;

    // Ger urdf
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

    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
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

    // Prepare robot interface
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

    if (quadrupedInterface->modelSettings().algorithm_ == tbai::mpc::quadruped::Algorithm::SQP) {
        TBAI_GLOBAL_LOG_INFO("Using SQP MPC");
        std::string sqpSettingsFile;
        TBAI_THROW_UNLESS(nodeHandle.getParam("/sqp_settings_file", sqpSettingsFile),
                          "Failed to get parameter /sqp_settings_file");
        const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
        auto mpcPtr = tbai::mpc::quadruped::getSqpMpc(*quadrupedInterface, mpcSettings, sqpSettings);
        tbai::mpc::quadruped::quadrupedMpcNode(robotName, nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    if (quadrupedInterface->modelSettings().algorithm_ == tbai::mpc::quadruped::Algorithm::DDP) {
        TBAI_GLOBAL_LOG_INFO("Using DDP MPC");
        const auto ddpSettings = ocs2::ddp::loadSettings(taskSettingsFile);
        auto mpcPtr = tbai::mpc::quadruped::getDdpMpc(*quadrupedInterface, mpcSettings, ddpSettings);
        tbai::mpc::quadruped::quadrupedMpcNode(robotName, nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    return EXIT_SUCCESS;
}
