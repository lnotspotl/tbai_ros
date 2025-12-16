#pragma once

namespace tbai {
namespace g1 {

// G1 29DOF joint count
constexpr int G1_NUM_JOINTS = 29;

// State vector size: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 29 joint pos + 29 joint vel
constexpr int G1_STATE_DIM = 3 + 3 + 3 + 3 + G1_NUM_JOINTS + G1_NUM_JOINTS;

// Observation history length
constexpr int HISTORY_LENGTH = 5;

// Observation dimensions per timestep (without history)
constexpr int OBS_BASE_ANG_VEL = 3;
constexpr int OBS_PROJECTED_GRAVITY = 3;
constexpr int OBS_VELOCITY_CMD = 3;
constexpr int OBS_JOINT_POS = G1_NUM_JOINTS;
constexpr int OBS_JOINT_VEL = G1_NUM_JOINTS;
constexpr int OBS_LAST_ACTION = G1_NUM_JOINTS;

// Total observation size with history
constexpr int OBS_SIZE_PER_STEP =
    OBS_BASE_ANG_VEL + OBS_PROJECTED_GRAVITY + OBS_VELOCITY_CMD + OBS_JOINT_POS + OBS_JOINT_VEL + OBS_LAST_ACTION;
constexpr int TOTAL_OBS_SIZE = OBS_SIZE_PER_STEP * HISTORY_LENGTH;

}  // namespace g1
}  // namespace tbai
