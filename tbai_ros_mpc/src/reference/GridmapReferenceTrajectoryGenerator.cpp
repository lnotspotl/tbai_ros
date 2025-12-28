#include "tbai_ros_mpc/reference/GridmapReferenceTrajectoryGenerator.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_filters_rsl/lookup.hpp>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/ReferenceExtrapolation.h>
#include <tbai_mpc/quadruped_mpc/quadruped_commands/TerrainAdaptation.h>
#include <tbai_mpc/quadruped_mpc/terrain/PlaneFitting.h>
#include <tbai_mpc/quadruped_mpc/core/Rotations.h>
#include <tbai_mpc/quadruped_mpc/core/SwitchedModel.h>

namespace tbai {
namespace mpc {
namespace reference {

using namespace switched_model;

namespace {
void addVelocitiesFromFiniteDifference(BaseReferenceTrajectory& baseRef) {
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
    baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[k + 1], baseRef.eulerXyz[k]) / dt);
    baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[k + 1] - baseRef.positionInWorld[k]) / dt);
  }

  auto dt = baseRef.time[N - 1] - baseRef.time[N - 2];
  baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[N - 1], baseRef.eulerXyz[N - 2]) / dt);
  baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[N - 1] - baseRef.positionInWorld[N - 2]) / dt);
}
}  // namespace

static BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
    const BaseReferenceCommand& command, const grid_map::GridMap& gridMap,
    double nominalStanceWidthInHeading, double nominalStanceWidthLateral) {
    const auto& baseReferenceLayer = gridMap.get("smooth_planar");

    // Helper to get a projected heading frame derived from the terrain.
    auto getLocalHeadingFrame = [&](const vector2_t& baseXYPosition, scalar_t yaw) {
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

    auto interp = [&](const vector2_t& offset) -> vector3_t {
    auto projection = grid_map::lookup::projectToMapWithMargin(gridMap, grid_map::Position(offset.x(), offset.y()));

    try {
    auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
    return vector3_t(offset.x(), offset.y(), z);
    } catch (std::out_of_range& e) {
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

    const auto normalAndPosition = estimatePlane({lfVerticalProjection, rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});

    TerrainPlane terrainPlane(normalAndPosition.position, orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
    return getProjectedHeadingFrame({0.0, 0.0, yaw}, terrainPlane);
    };

    auto reference2d = generate2DExtrapolatedBaseReference(horizon, initialState, command);

    BaseReferenceTrajectory baseRef;
    baseRef.time = std::move(reference2d.time);
    baseRef.eulerXyz.reserve(horizon.N);
    baseRef.positionInWorld.reserve(horizon.N);

    // Adapt poses
    for (int k = 0; k < horizon.N; ++k) {
    const auto projectedHeadingFrame = getLocalHeadingFrame(reference2d.positionInWorld[k], reference2d.yaw[k]);

    baseRef.positionInWorld.push_back(
    adaptDesiredPositionHeightToTerrain(reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
    baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain({0.0, 0.0, reference2d.yaw[k]}, projectedHeadingFrame));
    }

    addVelocitiesFromFiniteDifference(baseRef);
    return baseRef;
}


/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
GridmapReferenceTrajectoryGenerator::GridmapReferenceTrajectoryGenerator(
    ros::NodeHandle& nh,
    const std::string& configFile,
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr,
    std::shared_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> kinematicsPtr,
    ocs2::scalar_t trajdt,
    size_t trajKnots,
    const std::string& terrainTopic,
    bool blind)
    : ReferenceTrajectoryGenerator(configFile, std::move(velocityGeneratorPtr), std::move(kinematicsPtr), trajdt,
                                   trajKnots),
      blind_(blind) {
    if (!blind_) {
        terrainSubscriber_ = nh.subscribe(terrainTopic, 1, &GridmapReferenceTrajectoryGenerator::terrainCallback, this);
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::TargetTrajectories GridmapReferenceTrajectoryGenerator::generateReferenceTrajectory(
    ocs2::scalar_t currentTime, const ocs2::SystemObservation& observation) {
    // Update observation (this also updates terrain estimator if no external terrain)
    updateObservation(observation);

    // Check if we should use gridmap
    std::unique_lock<std::mutex> lock(terrainMutex_);
    bool useGridmap = !blind_ && terrainMapPtr_ != nullptr;

    BaseReferenceTrajectory baseReferenceTrajectory;

    if (useGridmap) {
        // Use gridmap-based reference generation
        baseReferenceTrajectory = generateExtrapolatedBaseReference(
            getBaseReferenceHorizon(), getBaseReferenceState(), getBaseReferenceCommand(currentTime), *terrainMapPtr_,
            nominalStanceWidthInHeading_, nominalStanceWidthLateral_);
    } else {
        lock.unlock();
        // Fall back to terrain plane-based generation
        baseReferenceTrajectory = generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                                                    getBaseReferenceCommand(currentTime),
                                                                    getTerrainPlane());
    }

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));

    for (size_t i = 0; i < N; ++i) {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = defaultJointState_;

        desiredStateTrajectory[i] = std::move(state);
    }

    return ocs2::TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                                    std::move(desiredInputTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool GridmapReferenceTrajectoryGenerator::hasGridmap() const {
    std::lock_guard<std::mutex> lock(terrainMutex_);
    return terrainMapPtr_ != nullptr;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void GridmapReferenceTrajectoryGenerator::terrainCallback(const grid_map_msgs::GridMap& msg) {
    if (!blind_) {
        // Convert ROS message to grid map
        std::unique_ptr<grid_map::GridMap> mapPtr(new grid_map::GridMap);
        std::vector<std::string> layers = {"smooth_planar"};
        grid_map::GridMapRosConverter::fromMessage(msg, *mapPtr, layers, false, false);

        // Swap terrain map pointers
        std::lock_guard<std::mutex> lock(terrainMutex_);
        terrainMapPtr_.swap(mapPtr);
    }
}

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
