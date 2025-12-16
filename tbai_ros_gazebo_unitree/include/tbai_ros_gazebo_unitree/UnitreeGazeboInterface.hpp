#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace gazebo {

class UnitreeGazeboInterface : public ModelPlugin {
   public:
    UnitreeGazeboInterface() {
        std::cerr << "[UnitreeGazeboInterface] Constructor called - library loaded!" << std::endl;
    }
    ~UnitreeGazeboInterface() override;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:
    void OnUpdate();
    void LowCmdCallback(const void *message);

    // Gazebo model and physics
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection_;

    // Joints
    std::vector<physics::JointPtr> joints_;
    std::unordered_map<std::string, int> jointIndexMap_;
    int numMotors_ = 0;

    // IMU sensor
    sensors::ImuSensorPtr imuSensor_;
    std::string imuSensorName_;

    // Unitree SDK2 DDS interface
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> lowstatePublisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmdSubscriber_;

    // Low command buffer (thread-safe)
    unitree_go::msg::dds_::LowCmd_ lowCmd_{};
    std::mutex lowCmdMutex_;

    // Low state message
    unitree_go::msg::dds_::LowState_ lowState_{};

    // Configuration
    int domainId_ = 1;              // Default to simulation domain
    std::string interface_ = "lo";  // Default to loopback for simulation
    double updateRate_ = 1000.0;    // Hz
    double updatePeriod_ = 0.001;   // seconds
    common::Time lastUpdateTime_;

    // Joint name mapping (configurable via SDF)
    std::vector<std::string> jointNames_;

    // Foot force sensors (optional)
    std::array<sensors::ContactSensorPtr, 4> footContactSensors_;
    std::array<std::string, 4> footContactSensorNames_;
    bool hasFootSensors_ = false;

    // State tracking
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{true};

    // Publish thread
    std::unique_ptr<std::thread> publishThread_;

    void PublishLoop();
    void UpdateLowState();
};

}  // namespace gazebo
