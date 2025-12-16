#include "tbai_ros_g1/MotionLoader.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tbai {
namespace g1 {

MotionLoader::MotionLoader(const std::string &motionFile, float fps)
    : dt_(1.0f / fps), index0_(0), index1_(0), blend_(0.0f) {
    loadCsv(motionFile);
    numFrames_ = static_cast<int>(dofPositions_.size());
    duration_ = numFrames_ * dt_;

    // Compute joint velocities from positions
    dofVelocities_ = computeRawDerivative(dofPositions_);

    // Initialize with identity transform
    worldToInit_.setIdentity();

    // Initialize interpolation at time 0
    update(0.0f);
}

void MotionLoader::loadCsv(const std::string &motionFile) {
    std::ifstream file(motionFile);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open motion file: " + motionFile);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::vector<float> values;
        std::stringstream ss(line);
        std::string token;

        while (std::getline(ss, token, ',')) {
            values.push_back(std::stof(token));
        }

        // Expected format: px, py, pz, qx, qy, qz, qw, joint0, ..., joint28
        // Total: 3 + 4 + 29 = 36 values
        if (values.size() < 36) {
            throw std::runtime_error("Invalid motion file format: expected at least 36 columns");
        }

        // Root position
        vector3_t rootPos(values[0], values[1], values[2]);
        rootPositions_.push_back(rootPos);

        // Root quaternion (qx, qy, qz, qw) -> Eigen convention (w, x, y, z)
        quaternion_t rootQuat(values[6], values[3], values[4], values[5]);
        rootQuat.normalize();
        rootQuaternions_.push_back(rootQuat);

        // Joint positions (29 DOF)
        vector_t jointPos(29);
        for (int i = 0; i < 29; ++i) {
            jointPos[i] = values[7 + i];
        }
        dofPositions_.push_back(jointPos);
    }

    if (dofPositions_.empty()) {
        throw std::runtime_error("Motion file is empty: " + motionFile);
    }
}

std::vector<vector_t> MotionLoader::computeRawDerivative(const std::vector<vector_t> &data) {
    std::vector<vector_t> derivative;
    derivative.reserve(data.size());

    for (size_t i = 0; i < data.size() - 1; ++i) {
        derivative.push_back((data[i + 1] - data[i]) / dt_);
    }

    // Last frame uses the same velocity as second-to-last
    if (!derivative.empty()) {
        derivative.push_back(derivative.back());
    } else {
        derivative.push_back(vector_t::Zero(29));
    }

    return derivative;
}

void MotionLoader::update(float time) {
    // Clamp time to valid range
    float phase = std::clamp(time / duration_, 0.0f, 1.0f);

    // Compute frame indices
    index0_ = static_cast<int>(std::round(phase * (numFrames_ - 1)));
    index1_ = std::min(index0_ + 1, numFrames_ - 1);

    // Compute blend factor with precision matching Unitree
    blend_ = std::round((time - index0_ * dt_) / dt_ * 1e5f) / 1e5f;
    blend_ = std::clamp(blend_, 0.0f, 1.0f);
}

void MotionLoader::reset(const quaternion_t &robotRootQuat, float startTime) {
    update(startTime);

    // Compute initial alignment between robot and motion
    matrix3_t initToAnchor = yawQuaternion(rootQuaternion()).toRotationMatrix();
    matrix3_t worldToAnchor = yawQuaternion(robotRootQuat).toRotationMatrix();
    worldToInit_ = worldToAnchor * initToAnchor.transpose();
}

vector_t MotionLoader::jointPos() const {
    return dofPositions_[index0_] * (1.0f - blend_) + dofPositions_[index1_] * blend_;
}

vector_t MotionLoader::jointVel() const {
    return dofVelocities_[index0_] * (1.0f - blend_) + dofVelocities_[index1_] * blend_;
}

vector3_t MotionLoader::rootPosition() const {
    return rootPositions_[index0_] * (1.0f - blend_) + rootPositions_[index1_] * blend_;
}

quaternion_t MotionLoader::rootQuaternion() const {
    return rootQuaternions_[index0_].slerp(blend_, rootQuaternions_[index1_]);
}

quaternion_t MotionLoader::yawQuaternion(const quaternion_t &quat) {
    // Extract yaw from quaternion and create yaw-only quaternion
    // Yaw is rotation around Z axis
    vector3_t euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order
    scalar_t yaw = euler[0];

    // Create quaternion from yaw only
    return quaternion_t(Eigen::AngleAxis<scalar_t>(yaw, vector3_t::UnitZ()));
}

}  // namespace g1
}  // namespace tbai
