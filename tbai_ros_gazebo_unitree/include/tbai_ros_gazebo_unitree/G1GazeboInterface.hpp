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
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace gazebo {

class G1GazeboInterface : public ModelPlugin {
   public:
    G1GazeboInterface() {
        std::cerr << "[G1GazeboInterface] Constructor called" << std::endl;
    }
    ~G1GazeboInterface() override;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:
    void OnUpdate();
    void LowCmdCallback(const void* message);

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

    // Unitree SDK2 DDS interface (using unitree_hg for G1)
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> lowstatePublisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> lowcmdSubscriber_;

    // Low command buffer (thread-safe)
    unitree_hg::msg::dds_::LowCmd_ lowCmd_{};
    std::mutex lowCmdMutex_;

    // Low state message
    unitree_hg::msg::dds_::LowState_ lowState_{};

    // Configuration
    int domainId_ = 1;
    std::string interface_ = "lo";
    double updateRate_ = 1000.0;
    double updatePeriod_ = 0.001;
    common::Time lastUpdateTime_;

    // Joint name mapping
    std::vector<std::string> jointNames_;

    // State tracking
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{true};

    // Initial positions for default position holding
    std::array<double, 29> initialPositions_{};
    bool initialPositionsSet_ = false;

    // Finite difference velocity calculation
    std::array<double, 29> prevPositions_{};
    std::array<double, 29> jointVelocities_{};
    double prevTime_ = 0.0;
    bool prevPositionsSet_ = false;

    // Publish thread
    std::unique_ptr<std::thread> publishThread_;

    void PublishLoop();
    void UpdateLowState();
};

}  // namespace gazebo
