#pragma once

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <tbai_ros_ocs2/mode_schedule.h>
#include <tbai_ros_ocs2/mpc_flattened_controller.h>
#include <tbai_ros_ocs2/mpc_observation.h>
#include <tbai_ros_ocs2/mpc_target_trajectories.h>
#include <tbai_ros_ocs2/reset.h>

#define PUBLISH_THREAD

namespace tbai {
namespace ocs2_ros {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 * Extended to support setTargetTrajectories and setModeSchedule via ROS topics.
 */
class MRT_ROS_Interface : public ::ocs2::MRT_BASE {
   public:
    /**
     * Constructor
     *
     * @param [in] topicPrefix: The prefix defines the names for: observation's publishing topic
     * "topicPrefix_mpc_observation", policy's receiving topic "topicPrefix_mpc_policy", and MPC reset service
     * "topicPrefix_mpc_reset".
     * @param [in] mrtTransportHints: ROS transmission protocol.
     */
    explicit MRT_ROS_Interface(std::string topicPrefix = "anonymousRobot",
                               ::ros::TransportHints mrtTransportHints = ::ros::TransportHints().tcpNoDelay());

    /**
     * Destructor
     */
    ~MRT_ROS_Interface() override;

    void resetMpcNode(const ::ocs2::TargetTrajectories &initTargetTrajectories) override;

    /**
     * Shut down the ROS nodes.
     */
    void shutdownNodes();

    /**
     * Shut down publisher
     */
    void shutdownPublisher();

    void spinMRT() override;
    /**
     * Launches the ROS publishers and subscribers to communicate with the MPC node.
     * @param nodeHandle
     */
    void launchNodes(::ros::NodeHandle &nodeHandle);

    void setCurrentObservation(const ::ocs2::SystemObservation &currentObservation) override;

    /**
     * @brief Sets target trajectories by publishing to the reference topic
     * @param targetTrajectories: the desired target trajectories
     */
    void setTargetTrajectories(const ::ocs2::TargetTrajectories &targetTrajectories) override;

    /**
     * @brief Sets mode schedule by publishing to the mode schedule topic
     * @param modeSchedule: the desired mode schedule
     */
    void setModeSchedule(const ::ocs2::ModeSchedule &modeSchedule) override;

   private:
    /**
     * Callback method to receive the MPC policy as well as the mode sequence.
     * It only updates the policy variables with suffix (*Buffer_) variables.
     *
     * @param [in] msg: A constant pointer to the message
     */
    void mpcPolicyCallback(const tbai_ros_ocs2::mpc_flattened_controller::ConstPtr &msg);

    /**
     * Helper function to read a MPC policy message.
     *
     * @param [in] msg: A constant pointer to the message
     * @param [out] commandData: The MPC command data
     * @param [out] primalSolution: The MPC policy data
     * @param [out] performanceIndices: The MPC performance indices data
     */
    static void readPolicyMsg(const tbai_ros_ocs2::mpc_flattened_controller &msg, ::ocs2::CommandData &commandData,
                              ::ocs2::PrimalSolution &primalSolution, ::ocs2::PerformanceIndex &performanceIndices);

    /**
     * A thread function which sends the current state and checks for a new MPC update.
     */
    void publisherWorkerThread();

   private:
    std::string topicPrefix_;

    // Publishers and subscribers
    ::ros::Publisher mpcObservationPublisher_;
    ::ros::Publisher targetTrajectoriesPublisher_;
    ::ros::Publisher modeSchedulePublisher_;
    ::ros::Subscriber mpcPolicySubscriber_;
    ::ros::ServiceClient mpcResetServiceClient_;

    // ROS messages
    tbai_ros_ocs2::mpc_observation mpcObservationMsg_;
    tbai_ros_ocs2::mpc_observation mpcObservationMsgBuffer_;

    ::ros::CallbackQueue mrtCallbackQueue_;
    ::ros::TransportHints mrtTransportHints_;

    // Multi-threading for publishers
    bool terminateThread_;
    bool readyToPublish_;
    std::thread publisherWorker_;
    std::mutex publisherMutex_;
    std::condition_variable msgReady_;
};

}  // namespace ocs2_ros
}  // namespace tbai
