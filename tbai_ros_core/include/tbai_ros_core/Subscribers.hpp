#pragma once

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "tbai_ros_msgs/RbdState.h"
#include "tbai_ros_msgs/RobotState.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tbai_estim/inekf/InEKFEstimator.hpp>
#include <tbai_estim/muse/MuseEstimator.hpp>
namespace tbai {

class RosStateSubscriber : public tbai::StateSubscriber {
   public:
    RosStateSubscriber(ros::NodeHandle &nh, const std::string &stateTopic);

    void waitTillInitialized() override;

    State getLatestState() override;

   private:
    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::RbdState::Ptr &msg);

    /** Shared pointer to the latest state message */
    tbai_ros_msgs::RbdState::Ptr stateMessage_;

    /** State message subscriber */
    ros::Subscriber stateSubscriber_;
};

class RosChangeControllerSubscriber : public ChangeControllerSubscriber {
   public:
    RosChangeControllerSubscriber(ros::NodeHandle &nh, const std::string &topic) {
        controllerSubscriber_ = nh.subscribe(topic, 1, &RosChangeControllerSubscriber::controllerCallback, this);
    }

    void triggerCallbacks() override {
        if (latestControllerType_.empty()) {
            return;
        }
        callbackFunction_(latestControllerType_);
        latestControllerType_.clear();
    }

   private:
    void controllerCallback(const std_msgs::String::ConstPtr &msg) { latestControllerType_ = msg->data; }

    ros::Subscriber controllerSubscriber_;
    std::string latestControllerType_;
};

class MuseRosStateSubscriber : public tbai::ThreadedStateSubscriber {
   public:
    MuseRosStateSubscriber(ros::NodeHandle &nhtemp, const std::string &stateTopic, const std::string &urdf = "");
    ~MuseRosStateSubscriber() override { stopThreads(); }
    void waitTillInitialized() override;

    void startThreads() override;
    void stopThreads() override;

   private:
    /** State message subscriber */
    ros::Subscriber stateSubscriber_;

    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg);

    void threadFunction();
    std::thread stateThread_;

    std::atomic_bool isRunning_ = false;
    std::atomic_bool isInitialized_ = false;

    ros::CallbackQueue thisQueue_;
    std::unique_ptr<tbai::muse::MuseEstimator> estimator_;

    std::shared_ptr<spdlog::logger> logger_;
    ros::Time lastStateTime_;
    bool firstState_ = true;
    scalar_t lastYaw_ = 0.0;
};

class InekfRosStateSubscriber : public tbai::ThreadedStateSubscriber {
   public:
    InekfRosStateSubscriber(ros::NodeHandle &nhtemp, const std::string &stateTopic, const std::string &urdf = "");
    ~InekfRosStateSubscriber() override { stopThreads(); }
    void waitTillInitialized() override;

    void startThreads() override;
    void stopThreads() override;

   private:
    /** State message subscriber */
    ros::Subscriber stateSubscriber_;

    /** State message callback */
    void stateMessageCallback(const tbai_ros_msgs::RobotState::Ptr &msg);

    void enable() override { enable_ = true; }
    void disable() override { enable_ = false; }

    bool enable_ = false;

    void threadFunction();
    std::thread stateThread_;

    std::atomic_bool isRunning_ = false;
    std::atomic_bool isInitialized_ = false;

    bool rectifyOrientation_ = true;
    bool removeGyroscopeBias_ = true;

    ros::CallbackQueue thisQueue_;
    std::unique_ptr<tbai::inekf::InEKFEstimator> estimator_;

    std::shared_ptr<spdlog::logger> logger_;
    ros::Time lastStateTime_;
    bool firstState_ = true;
    scalar_t lastYaw_ = 0.0;
};

}  // namespace tbai
