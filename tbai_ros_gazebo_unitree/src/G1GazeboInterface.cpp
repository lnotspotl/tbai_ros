#include "tbai_ros_gazebo_unitree/G1GazeboInterface.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace gazebo {

G1GazeboInterface::~G1GazeboInterface() {
    running_ = false;
    if (publishThread_ && publishThread_->joinable()) {
        publishThread_->join();
    }
}

void G1GazeboInterface::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    gzerr << "[G1GazeboInterface] Loading plugin..." << std::endl;

    model_ = model;

    // Read configuration from SDF
    if (sdf->HasElement("domain_id")) {
        domainId_ = sdf->Get<int>("domain_id");
    }
    if (sdf->HasElement("interface")) {
        interface_ = sdf->Get<std::string>("interface");
    }
    if (sdf->HasElement("update_rate")) {
        updateRate_ = sdf->Get<double>("update_rate");
        updatePeriod_ = 1.0 / updateRate_;
    }
    if (sdf->HasElement("imu_sensor")) {
        imuSensorName_ = sdf->Get<std::string>("imu_sensor");
    }

    gzerr << "[G1GazeboInterface] Domain ID: " << domainId_ << std::endl;
    gzerr << "[G1GazeboInterface] Interface: " << interface_ << std::endl;
    gzerr << "[G1GazeboInterface] Update rate: " << updateRate_ << " Hz" << std::endl;

    // Read joint names from SDF
    if (sdf->HasElement("joints")) {
        sdf::ElementPtr jointsElem = sdf->GetElement("joints");
        sdf::ElementPtr jointElem = jointsElem->GetElement("joint");
        while (jointElem) {
            std::string jointName = jointElem->Get<std::string>();
            jointNames_.push_back(jointName);
            jointElem = jointElem->GetNextElement("joint");
        }
    }

    // Get joints from model
    numMotors_ = jointNames_.size();
    if (numMotors_ == 0) {
        auto allJoints = model_->GetJoints();
        for (const auto& joint : allJoints) {
            if (joint->GetType() != physics::Joint::FIXED_JOINT) {
                jointNames_.push_back(joint->GetName());
            }
        }
        numMotors_ = jointNames_.size();
    }

    gzerr << "[G1GazeboInterface] Number of motors: " << numMotors_ << std::endl;

    // Get joint pointers
    for (int i = 0; i < numMotors_; ++i) {
        physics::JointPtr joint = model_->GetJoint(jointNames_[i]);
        if (joint) {
            joints_.push_back(joint);
            jointIndexMap_[jointNames_[i]] = i;
            gzerr << "[G1GazeboInterface] Joint " << i << ": " << jointNames_[i] << std::endl;
        } else {
            gzerr << "[G1GazeboInterface] Joint not found: " << jointNames_[i] << std::endl;
        }
    }

    // Initialize Unitree SDK2 DDS
    gzerr << "[G1GazeboInterface] Initializing DDS channel factory..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(domainId_, interface_);

    // Create publisher for LowState
    lowstatePublisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
    lowstatePublisher_->InitChannel();
    gzerr << "[G1GazeboInterface] LowState publisher initialized" << std::endl;

    // Create subscriber for LowCmd
    lowcmdSubscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
    lowcmdSubscriber_->InitChannel(
        std::bind(&G1GazeboInterface::LowCmdCallback, this, std::placeholders::_1), 1);
    gzerr << "[G1GazeboInterface] LowCmd subscriber initialized" << std::endl;

    // Initialize low command with safe defaults
    for (int i = 0; i < 29; ++i) {
        lowCmd_.motor_cmd()[i].mode() = 0x00;
        lowCmd_.motor_cmd()[i].q() = 0.0f;
        lowCmd_.motor_cmd()[i].dq() = 0.0f;
        lowCmd_.motor_cmd()[i].tau() = 0.0f;
        lowCmd_.motor_cmd()[i].kp() = 0.0f;
        lowCmd_.motor_cmd()[i].kd() = 0.0f;
    }

    // Connect to Gazebo update event
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&G1GazeboInterface::OnUpdate, this));

    // Initialize timing
    lastUpdateTime_ = model_->GetWorld()->SimTime();

    // Start publish thread
    publishThread_ = std::make_unique<std::thread>(&G1GazeboInterface::PublishLoop, this);

    initialized_ = true;
    gzerr << "[G1GazeboInterface] Plugin loaded successfully" << std::endl;
}

void G1GazeboInterface::LowCmdCallback(const void* message) {
    std::lock_guard<std::mutex> lock(lowCmdMutex_);
    lowCmd_ = *(unitree_hg::msg::dds_::LowCmd_*)message;
}

void G1GazeboInterface::OnUpdate() {
    std::lock_guard<std::mutex> lock(lowCmdMutex_);

    // Get current simulation time
    double currentTime = model_->GetWorld()->SimTime().Double();
    double dt = currentTime - prevTime_;

    // Default PD gains for position holding when not in servo mode
    constexpr double kp_default = 100.0;
    constexpr double kd_default = 10.0;

    for (int i = 0; i < numMotors_ && i < 29; ++i) {
        const auto& cmd = lowCmd_.motor_cmd()[i];

        double q_actual = joints_[i]->Position(0);

        // Calculate velocity using finite difference
        double dq_actual = 0.0;
        if (prevPositionsSet_ && dt > 1e-6) {
            dq_actual = (q_actual - prevPositions_[i]) / dt;
        }
        jointVelocities_[i] = dq_actual;

        double tau = 0.0;

        if (cmd.mode() == 0x01) {
            // Servo mode: apply commanded PD control with feedforward torque
            tau = cmd.tau() + cmd.kp() * (cmd.q() - q_actual) + cmd.kd() * (cmd.dq() - dq_actual);
        } else {
            // Idle mode: apply default position holding to prevent collapse
            // Hold at initial position with damping
            if (!initialPositionsSet_) {
                initialPositions_[i] = q_actual;
            }
            tau = kp_default * (initialPositions_[i] - q_actual) + kd_default * (0.0 - dq_actual);
        }

        joints_[i]->SetForce(0, tau);

        // Store current position for next iteration
        prevPositions_[i] = q_actual;
    }

    if (!initialPositionsSet_ && numMotors_ > 0) {
        initialPositionsSet_ = true;
    }

    prevPositionsSet_ = true;
    prevTime_ = currentTime;
}

void G1GazeboInterface::UpdateLowState() {
    // Update motor states (using finite difference velocities)
    for (int i = 0; i < numMotors_ && i < 29; ++i) {
        lowState_.motor_state()[i].q() = static_cast<float>(joints_[i]->Position(0));
        lowState_.motor_state()[i].dq() = static_cast<float>(jointVelocities_[i]);
        lowState_.motor_state()[i].tau_est() = static_cast<float>(joints_[i]->GetForce(0));
    }

    // Update IMU state from pelvis link
    physics::LinkPtr baseLink = model_->GetLink("pelvis");
    if (!baseLink) {
        baseLink = model_->GetLinks().front();
    }

    if (baseLink) {
        ignition::math::Pose3d pose = baseLink->WorldPose();
        ignition::math::Quaterniond orientation = pose.Rot();

        lowState_.imu_state().quaternion()[0] = static_cast<float>(orientation.W());
        lowState_.imu_state().quaternion()[1] = static_cast<float>(orientation.X());
        lowState_.imu_state().quaternion()[2] = static_cast<float>(orientation.Y());
        lowState_.imu_state().quaternion()[3] = static_cast<float>(orientation.Z());

        // RPY
        double w = orientation.W(), x = orientation.X(), y = orientation.Y(), z = orientation.Z();
        lowState_.imu_state().rpy()[0] = static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
        lowState_.imu_state().rpy()[1] = static_cast<float>(std::asin(2 * (w * y - z * x)));
        lowState_.imu_state().rpy()[2] = static_cast<float>(std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));

        // Angular velocity in body frame
        ignition::math::Vector3d angularVelWorld = baseLink->WorldAngularVel();
        ignition::math::Vector3d angularVelBody = orientation.Inverse().RotateVector(angularVelWorld);
        lowState_.imu_state().gyroscope()[0] = static_cast<float>(angularVelBody.X());
        lowState_.imu_state().gyroscope()[1] = static_cast<float>(angularVelBody.Y());
        lowState_.imu_state().gyroscope()[2] = static_cast<float>(angularVelBody.Z());

        // Linear acceleration in body frame
        ignition::math::Vector3d linearAccWorld = baseLink->WorldLinearAccel();
        linearAccWorld.Z() += 9.81;
        ignition::math::Vector3d linearAccBody = orientation.Inverse().RotateVector(linearAccWorld);
        lowState_.imu_state().accelerometer()[0] = static_cast<float>(linearAccBody.X());
        lowState_.imu_state().accelerometer()[1] = static_cast<float>(linearAccBody.Y());
        lowState_.imu_state().accelerometer()[2] = static_cast<float>(linearAccBody.Z());
    }

    // Timestamp
    common::Time simTime = model_->GetWorld()->SimTime();
    lowState_.tick() = static_cast<uint32_t>(simTime.Double() * 1000.0);
}

void G1GazeboInterface::PublishLoop() {
    using namespace std::chrono;
    auto period = microseconds(static_cast<int>(1000000.0 / updateRate_));

    while (running_) {
        auto start = high_resolution_clock::now();

        if (initialized_) {
            UpdateLowState();
            lowstatePublisher_->Write(lowState_);
        }

        auto elapsed = high_resolution_clock::now() - start;
        auto sleepTime = period - duration_cast<microseconds>(elapsed);
        if (sleepTime.count() > 0) {
            std::this_thread::sleep_for(sleepTime);
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(G1GazeboInterface)

}  // namespace gazebo
