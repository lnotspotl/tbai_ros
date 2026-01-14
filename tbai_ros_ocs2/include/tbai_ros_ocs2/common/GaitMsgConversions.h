/*
 * GaitMsgConversions.h
 *
 *  Created on: Jul 7, 2020
 *      Author: Oliver Harley, Marko Bjelonic, Ruben Grandia
 */

#pragma once

#include <tbai_mpc/quadruped_mpc/logic/Gait.h>
#include <tbai_mpc/quadruped_mpc/logic/GaitSchedule.h>

#include <tbai_ros_ocs2/gait.h>
#include <tbai_ros_ocs2/gait_sequence.h>
#include <tbai_ros_ocs2/scheduled_gait_sequence.h>

namespace tbai::mpc::quadruped {
namespace ros_msg_conversions {

tbai_ros_ocs2::gait toMessage(const Gait& gait);
Gait fromMessage(const tbai_ros_ocs2::gait& msg);

tbai_ros_ocs2::gait_sequence toMessage(const GaitSchedule::GaitSequence& gaitSequence);
GaitSchedule::GaitSequence fromMessage(const tbai_ros_ocs2::gait_sequence& msg);

tbai_ros_ocs2::scheduled_gait_sequence toMessage(scalar_t startTime, const GaitSchedule::GaitSequence& gaitSequence);
std::pair<scalar_t, GaitSchedule::GaitSequence> fromMessage(const tbai_ros_ocs2::scheduled_gait_sequence& msg);

}  // namespace ros_msg_conversions
}  // namespace tbai::mpc::quadruped
