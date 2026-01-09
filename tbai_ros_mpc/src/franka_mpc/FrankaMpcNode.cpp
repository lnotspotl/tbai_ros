#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ros/init.h>
#include <ros/package.h>
#include <tbai_mpc/franka_mpc/FrankaInterface.h>
#include <tbai_ros_ocs2/MPC_ROS_Interface.hpp>
#include <tbai_ros_ocs2/RosReferenceManager.hpp>

using namespace ocs2;
using namespace franka;

int main(int argc, char **argv) {
    const std::string robotName = "franka";

    ros::init(argc, argv, robotName + "_mpc");
    ros::NodeHandle nodeHandle;

    std::string taskFile, libFolder, urdfFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/libFolder", libFolder);
    nodeHandle.getParam("/urdfFile", urdfFile);
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;

    FrankaInterface interface(taskFile, libFolder, urdfFile);

    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);

    GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                           interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

    MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(nodeHandle);

    return 0;
}
