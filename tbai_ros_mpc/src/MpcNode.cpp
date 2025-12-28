#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ros/init.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/QuadrupedMpc.h>
#include <tbai_mpc/quadruped_mpc/anymal_interface/Interface.h>
#include <tbai_ros_mpc/QuadrupedMpcNode.h>


namespace switched_model {

std::unique_ptr<ocs2::MPC_BASE> getDdpMpc(const QuadrupedInterface &quadrupedInterface,
                                          const ocs2::mpc::Settings &mpcSettings,
                                          const ocs2::ddp::Settings &ddpSettings) {
    std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::GaussNewtonDDP_MPC(
        mpcSettings, ddpSettings, quadrupedInterface.getRollout(), quadrupedInterface.getOptimalControlProblem(),
        quadrupedInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->setSynchronizedModules(quadrupedInterface.getSynchronizedModules());
    return mpcPtr;
}

std::unique_ptr<ocs2::MPC_BASE> getSqpMpc(const QuadrupedInterface &quadrupedInterface,
                                          const ocs2::mpc::Settings &mpcSettings,
                                          const ocs2::sqp::Settings &sqpSettings) {
    std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::SqpMpc(
        mpcSettings, sqpSettings, quadrupedInterface.getOptimalControlProblem(), quadrupedInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
    mpcPtr->getSolverPtr()->setSynchronizedModules(quadrupedInterface.getSynchronizedModules());
    return mpcPtr;
}

}  // namespace switched_model

using namespace switched_model;

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nodeHandle;

    // Anymal urdf
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
    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterface;
    if (robotName == "anymal_d") {
        quadrupedInterface =
            anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                       anymal::frameDeclarationFromFile(frameDeclarationFile));
        // } else if (robotName == "go2") {
        //     quadrupedInterface =
        //         anymal::getGo2Interface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
        //                                 anymal::frameDeclarationFromFile(frameDeclarationFile));
    } else {
        TBAI_THROW("Robot name not supported: {}", robotName);
    }

    // Prepare robot interface
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

    if (quadrupedInterface->modelSettings().algorithm_ == switched_model::Algorithm::SQP) {
        TBAI_GLOBAL_LOG_INFO("Using SQP MPC");
        std::string sqpSettingsFile;
        TBAI_THROW_UNLESS(nodeHandle.getParam("/sqp_settings_file", sqpSettingsFile),
                          "Failed to get parameter /sqp_settings_file");
        const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
        auto mpcPtr = switched_model::getSqpMpc(*quadrupedInterface, mpcSettings, sqpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    if (quadrupedInterface->modelSettings().algorithm_ == switched_model::Algorithm::DDP) {
        TBAI_GLOBAL_LOG_INFO("Using DDP MPC");
        const auto ddpSettings = ocs2::ddp::loadSettings(taskSettingsFile);
        auto mpcPtr = switched_model::getDdpMpc(*quadrupedInterface, mpcSettings, ddpSettings);
        switched_model::quadrupedMpcNode(nodeHandle, *quadrupedInterface, std::move(mpcPtr));
    }

    return EXIT_SUCCESS;
}
