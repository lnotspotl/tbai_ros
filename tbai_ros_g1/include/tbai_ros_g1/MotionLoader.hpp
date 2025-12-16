#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tbai_core/Types.hpp>

namespace tbai {
namespace g1 {

class MotionLoader {
   public:
    MotionLoader(const std::string &motionFile, float fps);

    void update(float time);

    void reset(const quaternion_t &robotRootQuat, float startTime = 0.0f);
    vector_t jointPos() const;

    vector_t jointVel() const;

    vector3_t rootPosition() const;

    quaternion_t rootQuaternion() const;

    static quaternion_t yawQuaternion(const quaternion_t &quat);

    float dt() const { return dt_; }
    int numFrames() const { return numFrames_; }
    float duration() const { return duration_; }
    const matrix3_t &worldToInit() const { return worldToInit_; }

   private:
    void loadCsv(const std::string &motionFile);
    std::vector<vector_t> computeRawDerivative(const std::vector<vector_t> &data);

    float dt_;
    int numFrames_;
    float duration_;

    // Motion data
    std::vector<vector3_t> rootPositions_;
    std::vector<quaternion_t> rootQuaternions_;
    std::vector<vector_t> dofPositions_;
    std::vector<vector_t> dofVelocities_;

    // Interpolation state
    int index0_;
    int index1_;
    float blend_;

    // Initial alignment transform
    matrix3_t worldToInit_;
};

}  // namespace g1
}  // namespace tbai
