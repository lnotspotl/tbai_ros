#pragma once

#include <ros/ros.h>
#include <tbai_core/Types.hpp>

namespace tbai {
struct RosTime {
    static inline scalar_t rightNow() { return ros::Time::now().toSec(); }
};
}  // namespace tbai
