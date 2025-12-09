#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_gridmap/GridmapInterface.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

// Perceptive Joe
#include <ocs2_anymal_models/FrameDeclaration.h>

// This is a bit hacky :/
#define private public
#define protected public
#include <ocs2_anymal_models/QuadrupedCom.h>
#undef private
#undef protected
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include <ocs2_anymal_mpc/AnymalInterface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/control/Controllers.hpp>

namespace tbai {
namespace joe {

using ocs2::SystemObservation;
using ocs2::TargetTrajectories;
using ocs2::legged_robot::contact_flag_t;
using switched_model::BaseReferenceCommand;
using switched_model::BaseReferenceHorizon;
using switched_model::BaseReferenceState;

class JoeController final : public tbai::Controller {
   public:
    JoeController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriber);

    std::vector<MotorCommand> getMotorCommands(scalar_t currentTime, scalar_t dt) override;

    void postStep(scalar_t currentTime, scalar_t dt) override;

    void changeController(const std::string &controllerType, scalar_t currentTime) override;

    bool isSupported(const std::string &controllerType) override;

    void stopController() override {}

    scalar_t getRate() const override { return 50.0; }

    bool checkStability() const override;

    bool ok() const override { return ros::ok(); }

    void waitTillInitialized() override { stateSubscriberPtr_->waitTillInitialized(); }

    std::string getName() const override { return "JoeController"; }

    void preStep(scalar_t currentTime, scalar_t dt) override {
        ros::spinOnce();
        state_ = stateSubscriberPtr_->getLatestState();
    }

   private:
    // Torchscript model
    torch::jit::script::Module joeModel_;

    ocs2::SystemObservation generateSystemObservation();

    // Helper functions
    inline vector_t getRpyAngles(const vector_t &state) const {
        vector_t rpyocs2 = state.head<3>();
        return tbai::mat2rpy(tbai::ocs2rpy2quat(rpyocs2).toRotationMatrix());
    }
    inline vector_t getOcs2ZyxEulerAngles(const vector_t &state) {
        vector_t rpy = getRpyAngles(state);
        rpy(2) = ocs2::moduloAngleWithReference(rpy(2), yawLast_);
        yawLast_ = rpy(2);
        return rpy.reverse();
    }
    inline matrix3_t getRotationMatrixWorldBase(const vector_t &state) const {
        return tbai::ocs2rpy2quat(state.head<3>()).toRotationMatrix();
    }
    inline matrix3_t getRotationMatrixBaseWorld(const vector_t &state) const {
        return getRotationMatrixWorldBase(state).transpose();
    }
    inline matrix3_t getRotationMatrixWorldBaseYaw(const vector_t &state) const {
        vector_t rpy = getRpyAngles(state);
        rpy.head<2>().setZero();
        return tbai::rpy2mat(rpy);
    }
    inline matrix3_t getRotationMatrixBaseWorldYaw(const vector_t &state) const {
        return getRotationMatrixWorldBaseYaw(state).transpose();
    }
    inline quaternion_t getQuaternionFromEulerAnglesZyx(const vector3_t &eulerAnglesZyx) const {
        return ocs2::getQuaternionFromEulerAnglesZyx(eulerAnglesZyx);
    }

    void resetMpc();
    void setObservation();

    // Helper functions
    contact_flag_t getDesiredContactFlags(scalar_t currentTime, scalar_t dt);
    vector_t getTimeLeftInPhase(scalar_t currentTime, scalar_t dt);
    TargetTrajectories generateTargetTrajectories(scalar_t currentTime, scalar_t dt, const vector3_t &command);
    std::vector<vector3_t> getCurrentFeetPositions(scalar_t currentTime, scalar_t dt);
    std::vector<vector3_t> getCurrentFeetVelocities(scalar_t currentTime, scalar_t dt);
    std::vector<vector3_t> getDesiredFeetPositions(scalar_t currentTime, scalar_t dt);
    std::vector<vector3_t> getDesiredFeetVelocities(scalar_t currentTime, scalar_t dt);
    void computeBaseKinematicsAndDynamics(scalar_t currentTime, scalar_t dt, vector3_t &basePos,
                                          vector_t &baseOrientation, vector3_t &baseLinearVelocity,
                                          vector3_t &baseAngularVelocity, vector3_t &baseLinearAcceleration,
                                          vector3_t &baseAngularAcceleration);

    // Observations
    vector3_t getLinearVelocityObservation(scalar_t currentTime, scalar_t dt) const;
    vector3_t getAngularVelocityObservation(scalar_t currentTime, scalar_t dt) const;
    vector3_t getProjectedGravityObservation(scalar_t currentTime, scalar_t dt) const;
    vector3_t getCommandObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDofPosObservation(scalar_t currentTime, scalar_t dt) const;
    vector_t getDofVelObservation(scalar_t currentTime, scalar_t dt) const;
    vector_t getPastActionObservation(scalar_t currentTime, scalar_t dt) const;
    vector_t getPlanarFootholdsObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredJointAnglesObservation(scalar_t currentTime, scalar_t dt);
    vector_t getCurrentDesiredJointAnglesObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredContactsObservation(scalar_t currentTime, scalar_t dt);
    vector_t getTimeLeftInPhaseObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredBasePosObservation(scalar_t currentTime, scalar_t dt);
    vector_t getOrientationDiffObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredBaseLinVelObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredBaseAngVelObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredBaseLinAccObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredBaseAngAccObservation(scalar_t currentTime, scalar_t dt);
    vector_t getCpgObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredFootPositionsObservation(scalar_t currentTime, scalar_t dt);
    vector_t getDesiredFootVelocitiesObservation(scalar_t currentTime, scalar_t dt);
    vector_t getHeightSamplesObservation(scalar_t currentTime, scalar_t dt);

    std::shared_ptr<tbai::StateSubscriber> stateSubscriberPtr_;

    const scalar_t LIN_VEL_SCALE = 2.0;
    const scalar_t ANG_VEL_SCALE = 0.25;
    const scalar_t COMMAND_SCALE = 1.0;
    const scalar_t GRAVITY_SCALE = 1.0;
    const scalar_t DOF_POS_SCALE = 1.0;
    const scalar_t DOF_VEL_SCALE = 0.05;
    const scalar_t PAST_ACTION_SCALE = 1.0;

    const scalar_t ISAAC_SIM_DT = 1 / 50;

    const int64_t MODEL_INPUT_SIZE = 179;
    // const long MODEL_INPUT_SIZE = 124;

    std::unique_ptr<TargetTrajectories> lastTargetTrajectories_;

    scalar_t yawLast_ = 0.0;

    // Use this to multiply the NN output before using it to generate the command message
    const scalar_t ACTION_SCALE = 0.5;

    scalar_t initTime_;

    ocs2::MRT_ROS_Interface mrt_;
    bool mrt_initialized_ = false;

    std::unique_ptr<tbai::reference::ReferenceVelocityGenerator> refVelGen_;
    void publishReference(const TargetTrajectories &targetTrajectories);
    ros::Publisher refPub_;

    vector_t defaultJointAngles_;
    std::vector<std::string> jointNames_;

    vector_t pastAction_;

    scalar_t horizon_;
    scalar_t mpcRate_ = 2.5;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;
    scalar_t timeSinceLastMpcUpdate2_ = 1e5;

    bool blind_ = false;
    std::unique_ptr<tbai::gridmap::GridmapInterface> gridmap_;
    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterface_;
    std::unique_ptr<switched_model::ComModelBase<scalar_t>> comModel_;
    std::unique_ptr<switched_model::KinematicsModelBase<scalar_t>> kinematicsModel_;
    std::unique_ptr<switched_model::QuadrupedVisualizer> visualizer_;

    BaseReferenceHorizon getBaseReferenceHorizon(scalar_t time) { return {0.1, 10}; }

    BaseReferenceState getBaseReferenceState(scalar_t time) {
        auto currentObservation = generateSystemObservation();
        auto observationTime = currentObservation.time;

        Eigen::Vector3d positionInWorld;
        if (lastTargetTrajectories_.get() == nullptr) {
            positionInWorld = currentObservation.state.segment<3>(3);
        } else {
            positionInWorld = lastTargetTrajectories_->getDesiredState(time).segment<3>(3);

            auto currentPos = currentObservation.state.segment<3>(3);
            if ((positionInWorld - currentPos).norm() > 0.1) {
                positionInWorld = currentPos;
            }
        }

        Eigen::Vector3d eulerXyz = currentObservation.state.head<3>(0);
        return {observationTime, positionInWorld, eulerXyz};
    }

    BaseReferenceCommand getBaseReferenceCommand(scalar_t time, const vector_t &command) {
        scalar_t vx = command(0) / LIN_VEL_SCALE;
        scalar_t vy = command(1) / LIN_VEL_SCALE;
        scalar_t wz = command(2) / ANG_VEL_SCALE;
        scalar_t comHeight = 0.53;
        return {vx, vy, wz, comHeight};
    }

    std::shared_ptr<spdlog::logger> logger_;

    State state_;
};

/** Torch -> Eigen*/
vector_t torch2vector(const torch::Tensor &t);
matrix_t torch2matrix(const torch::Tensor &t);

/** Eigen -> Torch */
torch::Tensor vector2torch(const vector_t &v);
torch::Tensor matrix2torch(const matrix_t &m);

}  // namespace joe
}  // namespace tbai
