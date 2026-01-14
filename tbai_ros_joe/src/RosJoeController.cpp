// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_joe/RosJoeController.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_filters_rsl/lookup.hpp>
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/ReferenceExtrapolation.h>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/TerrainAdaptation.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlaneFitting.h>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_ocs2/common/RosMsgConversions.h>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace joe {

using namespace tbai::mpc::quadruped;

namespace {
void addVelocitiesFromFiniteDifference(BaseReferenceTrajectory &baseRef) {
    auto N = baseRef.time.size();
    if (N <= 1) {
        return;
    }

    baseRef.linearVelocityInWorld.clear();
    baseRef.angularVelocityInWorld.clear();
    baseRef.linearVelocityInWorld.reserve(N);
    baseRef.angularVelocityInWorld.reserve(N);

    for (size_t k = 0; (k + 1) < baseRef.time.size(); ++k) {
        auto dt = baseRef.time[k + 1] - baseRef.time[k];
        baseRef.angularVelocityInWorld.push_back(
            rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[k + 1], baseRef.eulerXyz[k]) / dt);
        baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[k + 1] - baseRef.positionInWorld[k]) / dt);
    }

    auto dt = baseRef.time[N - 1] - baseRef.time[N - 2];
    baseRef.angularVelocityInWorld.push_back(
        rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[N - 1], baseRef.eulerXyz[N - 2]) / dt);
    baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[N - 1] - baseRef.positionInWorld[N - 2]) / dt);
}

BaseReferenceTrajectory generateExtrapolatedBaseReferenceFromGridmap(
    const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState, const BaseReferenceCommand &command,
    const grid_map::GridMap &gridMap, double nominalStanceWidthInHeading, double nominalStanceWidthLateral) {
    const auto &baseReferenceLayer = gridMap.get("smooth_planar");

    // Helper to get a projected heading frame derived from the terrain.
    auto getLocalHeadingFrame = [&](const vector2_t &baseXYPosition, scalar_t yaw) {
        vector2_t lfOffset(nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0);
        vector2_t rfOffset(nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0);
        vector2_t lhOffset(-nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0);
        vector2_t rhOffset(-nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0);
        // Rotate from heading to world frame
        rotateInPlace2d(lfOffset, yaw);
        rotateInPlace2d(rfOffset, yaw);
        rotateInPlace2d(lhOffset, yaw);
        rotateInPlace2d(rhOffset, yaw);
        // shift by base center
        lfOffset += baseXYPosition;
        rfOffset += baseXYPosition;
        lhOffset += baseXYPosition;
        rhOffset += baseXYPosition;

        auto interp = [&](const vector2_t &offset) -> vector3_t {
            auto projection =
                grid_map::lookup::projectToMapWithMargin(gridMap, grid_map::Position(offset.x(), offset.y()));

            try {
                auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
                return vector3_t(offset.x(), offset.y(), z);
            } catch (std::out_of_range &e) {
                double interp = gridMap.getResolution() / (projection - gridMap.getPosition()).norm();
                projection = (1.0 - interp) * projection + interp * gridMap.getPosition();
                auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
                return vector3_t(offset.x(), offset.y(), z);
            }
        };

        vector3_t lfVerticalProjection = interp(lfOffset);
        vector3_t rfVerticalProjection = interp(rfOffset);
        vector3_t lhVerticalProjection = interp(lhOffset);
        vector3_t rhVerticalProjection = interp(rhOffset);

        const auto normalAndPosition =
            estimatePlane({lfVerticalProjection, rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});

        TerrainPlane terrainPlane(normalAndPosition.position,
                                  orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
        return getProjectedHeadingFrame({0.0, 0.0, yaw}, terrainPlane);
    };

    auto reference2d = generate2DExtrapolatedBaseReference(horizon, initialState, command);

    BaseReferenceTrajectory baseRef;
    baseRef.time = std::move(reference2d.time);
    baseRef.eulerXyz.reserve(horizon.N);
    baseRef.positionInWorld.reserve(horizon.N);

    // Adapt poses
    for (size_t k = 0; k < horizon.N; ++k) {
        const auto projectedHeadingFrame = getLocalHeadingFrame(reference2d.positionInWorld[k], reference2d.yaw[k]);

        baseRef.positionInWorld.push_back(adaptDesiredPositionHeightToTerrain(
            reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
        baseRef.eulerXyz.emplace_back(
            alignDesiredOrientationToTerrain({0.0, 0.0, reference2d.yaw[k]}, projectedHeadingFrame));
    }

    addVelocitiesFromFiniteDifference(baseRef);
    return baseRef;
}
}  // namespace

TargetTrajectories GridmapTerrainInterface::generateTargetTrajectories(
    scalar_t currentTime, const BaseReferenceHorizon &horizon, const BaseReferenceState &state,
    const BaseReferenceCommand &command, const tbai::mpc::quadruped::QuadrupedInterface &quadrupedInterface) {
    tbai::mpc::quadruped::BaseReferenceTrajectory baseReferenceTrajectory =
        generateExtrapolatedBaseReferenceFromGridmap(horizon, state, command, gridmap_->getMap(), 0.3, 0.3);

    constexpr size_t STATE_DIM = 6 + 6 + 12;
    constexpr size_t INPUT_DIM = 12 + 12;

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));
    for (size_t i = 0; i < N; ++i) {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = tbai::mpc::quadruped::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = quadrupedInterface.getInitialState().segment<12>(12);

        desiredStateTrajectory[i] = std::move(state);
    }

    return TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                              std::move(desiredInputTrajectory));
}

RosJoeController::RosJoeController(const std::string &robotName,
                                   const std::shared_ptr<tbai::StateSubscriber> &stateSubscriber,
                                   std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGenerator,
                                   std::function<scalar_t()> getCurrentTimeFunction)
    : JoeController(robotName, stateSubscriber, std::move(velocityGenerator), std::move(getCurrentTimeFunction)) {
    // Load parameters from ROS parameter server
    std::string taskFile, urdfFile, referenceFile;
    TBAI_THROW_UNLESS(nh_.getParam("taskFile", taskFile), "Task file not found");
    TBAI_THROW_UNLESS(nh_.getParam("urdfFile", urdfFile), "URDF file not found");
    TBAI_THROW_UNLESS(nh_.getParam("referenceFile", referenceFile), "Reference file not found");

    // Get URDF from parameter server
    std::string urdf, taskSettingsFile, frameDeclarationFile;
    ros::param::get("/robot_description", urdf);
    ros::param::get("/task_settings_file", taskSettingsFile);
    ros::param::get("/frame_declaration_file", frameDeclarationFile);

    // Load JOE model
    std::string hfRepo = tbai::fromGlobalConfig<std::string>("joe_controller/hf_repo");
    std::string hfModel = tbai::fromGlobalConfig<std::string>("joe_controller/hf_model");
    std::string modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);

    // Initialize reference publisher
    refPub_ = nh_.advertise<tbai_ros_ocs2::mpc_target_trajectories>(robotName_ + "_mpc_target", 1, false);

    // Check if blind mode
    bool blind = tbai::fromGlobalConfig<bool>("joe_controller/blind");
    if (!blind) {
        TBAI_LOG_INFO(logger_, "Initializing gridmap interface for perceptive mode");
        setTerrainInterface(std::make_unique<GridmapTerrainInterface>());
    } else {
        TBAI_LOG_INFO(logger_, "Running in blind mode");
    }

    // Initialize the base controller
    initialize(urdf, taskSettingsFile, frameDeclarationFile, modelPath);

    // Initialize visualizer after quadruped interface is ready
    visualizer_.reset(new tbai::mpc::quadruped::QuadrupedVisualizer(quadrupedInterface_->getKinematicModel(),
                                                                    quadrupedInterface_->getJointNames(),
                                                                    quadrupedInterface_->getBaseName(), nh_));
}

std::unique_ptr<ocs2::MRT_BASE> RosJoeController::createMrtInterface() {
    // Create ROS-based MRT interface
    auto mrtRos = std::make_unique<tbai::ocs2_ros::MRT_ROS_Interface>(robotName_);
    mrtRos->launchNodes(nh_);
    return mrtRos;
}

std::unique_ptr<ocs2::MPC_BASE> RosJoeController::createMpcInterface() {
    // MPC runs in separate node when using ROS interface
    return nullptr;
}

void RosJoeController::publishReference(const TargetTrajectories &targetTrajectories) {
    const auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
    refPub_.publish(mpcTargetTrajectoriesMsg);
}

void RosJoeController::postStep(scalar_t currentTime, scalar_t dt) {
    visualizer_->update(generateSystemObservation(), mrt_->getPolicy(), mrt_->getCommand());
}

void RosJoeController::resetMpc() {
    TBAI_LOG_INFO(logger_, "Waiting for state subscriber to initialize...");

    // Wait to receive observation
    stateSubscriberPtr_->waitTillInitialized();

    TBAI_LOG_INFO(logger_, "State subscriber initialized");

    // Prepare initial observation for MPC
    ocs2::SystemObservation mpcObservation = generateSystemObservation();

    TBAI_LOG_INFO(logger_, "Initial observation generated");

    // Prepare target trajectory
    ocs2::TargetTrajectories initTargetTrajectories({0.0}, {mpcObservation.state}, {mpcObservation.input});

    TBAI_LOG_INFO(logger_, "Resetting MPC...");

    // Use the ROS-specific reset
    auto *mrtRos = dynamic_cast<tbai::ocs2_ros::MRT_ROS_Interface *>(mrt_.get());
    if (mrtRos != nullptr) {
        mrtRos->resetMpcNode(initTargetTrajectories);

        while (!mrt_->initialPolicyReceived() && ros::ok()) {
            TBAI_LOG_INFO(logger_, "Waiting for initial policy...");
            ros::spinOnce();
            mrtRos->spinMRT();
            mrt_->setCurrentObservation(generateSystemObservation());
            ros::Duration(0.1).sleep();
        }
    }

    TBAI_LOG_INFO(logger_, "Initial policy received");
}

}  // namespace joe
}  // namespace tbai
