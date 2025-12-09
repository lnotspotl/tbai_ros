
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_mpc/reference/ReferenceTrajectoryGenerator.hpp>
#include <tbai_ros_mpc/wbc/WbcBase.hpp>
#include <tbai_ros_msgs/JointCommandArray.h>

namespace tbai {

namespace mpc {

class ContactVisualizer {
   public:
    ContactVisualizer();
    void visualize(const vector_t &currentState, const std::vector<bool> &contacts);

   private:
    /** Odom frame name */
    std::string odomFrame_;

    /** Base frame name */
    ros::Publisher contactPublisher_;

    /** List of foot frame names */
    std::vector<std::string> footFrameNames_;

    pinocchio::Model model_;
    pinocchio::Data data_;
};

class MpcController final : public tbai::Controller {
   public:
    MpcController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                  std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr);

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override;

    std::string getName() const override { return "MpcController"; }

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override { stopReferenceThread(); }

    scalar_t getRate() const override { return 400.0; }

    bool checkStability() const override;

    bool ok() const override { return ros::ok(); }

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    void preStep(scalar_t currentTime, scalar_t dt) override {
        ros::spinOnce();
        state_ = stateSubscriberPtr_->getLatestState();
    }

   private:
    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterfacePtr_;
    std::unique_ptr<switched_model::QuadrupedVisualizer> visualizerPtr_;
    std::unique_ptr<ContactVisualizer> contactVisualizerPtr_;
    std::unique_ptr<switched_model::WbcBase> wbcPtr_;
    std::unique_ptr<reference::ReferenceTrajectoryGenerator> referenceTrajectoryGeneratorPtr_;

    void referenceThread();
    void spinOnceReferenceThread();
    ros::NodeHandle referenceThreadNodeHandle_;
    ros::CallbackQueue referenceThreadCallbackQueue_;
    void startReferenceThread();
    void stopReferenceThread();

    std::atomic<bool> stopReferenceThread_;
    std::thread referenceThread_;

    void resetMpc();

    void setObservation();

    ocs2::SystemObservation generateSystemObservation() const;

    scalar_t initTime_;
    scalar_t tNow_;

    ocs2::MRT_ROS_Interface mrt_;
    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 30.0;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;
    scalar_t timeSinceLastVisualizationUpdate_ = 1e5;

    bool isStable_ = true;

    State state_;

    std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace mpc
}  // namespace tbai
