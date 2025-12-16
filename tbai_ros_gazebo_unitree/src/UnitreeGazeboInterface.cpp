#include "tbai_ros_gazebo_unitree/UnitreeGazeboInterface.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace gazebo {

UnitreeGazeboInterface::~UnitreeGazeboInterface() {
    running_ = false;
    if (publishThread_ && publishThread_->joinable()) {
        publishThread_->join();
    }
}

void UnitreeGazeboInterface::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
    gzerr << "[UnitreeGazeboInterface] Loading plugin..." << std::endl;

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

    gzerr << "[UnitreeGazeboInterface] Domain ID: " << domainId_ << std::endl;
    gzerr << "[UnitreeGazeboInterface] Interface: " << interface_ << std::endl;
    gzerr << "[UnitreeGazeboInterface] Update rate: " << updateRate_ << " Hz" << std::endl;

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

    // Read foot contact sensor names from SDF (optional)
    if (sdf->HasElement("foot_sensors")) {
        sdf::ElementPtr footSensorsElem = sdf->GetElement("foot_sensors");
        std::array<std::string, 4> footTags = {"FR", "FL", "RR", "RL"};
        for (int i = 0; i < 4; ++i) {
            if (footSensorsElem->HasElement(footTags[i])) {
                footContactSensorNames_[i] = footSensorsElem->Get<std::string>(footTags[i]);
                hasFootSensors_ = true;
            }
        }
    }

    // Get joints from model
    numMotors_ = jointNames_.size();
    if (numMotors_ == 0) {
        // If no joints specified in SDF, get all joints from model
        auto allJoints = model_->GetJoints();
        for (const auto &joint : allJoints) {
            // Skip fixed joints
            if (joint->GetType() != physics::Joint::FIXED_JOINT) {
                jointNames_.push_back(joint->GetName());
            }
        }
        numMotors_ = jointNames_.size();
    }

    gzerr << "[UnitreeGazeboInterface] Number of motors: " << numMotors_ << std::endl;

    // Get joint pointers and create index map
    for (int i = 0; i < numMotors_; ++i) {
        physics::JointPtr joint = model_->GetJoint(jointNames_[i]);
        if (joint) {
            joints_.push_back(joint);
            jointIndexMap_[jointNames_[i]] = i;
            gzerr << "[UnitreeGazeboInterface] Joint " << i << ": " << jointNames_[i] << std::endl;
        } else {
            gzerr << "[UnitreeGazeboInterface] Joint not found: " << jointNames_[i] << std::endl;
        }
    }

    // Get IMU sensor
    if (!imuSensorName_.empty()) {
        sensors::SensorManager *sensorManager = sensors::SensorManager::Instance();
        sensors::SensorPtr sensor = sensorManager->GetSensor(imuSensorName_);
        if (sensor) {
            imuSensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(sensor);
            if (imuSensor_) {
                gzerr << "[UnitreeGazeboInterface] IMU sensor found: " << imuSensorName_ << std::endl;
            } else {
                gzerr << "[UnitreeGazeboInterface] Sensor is not an IMU: " << imuSensorName_ << std::endl;
            }
        } else {
            gzerr << "[UnitreeGazeboInterface] IMU sensor not found: " << imuSensorName_ << std::endl;
        }
    }

    // Get foot contact sensors (optional)
    if (hasFootSensors_) {
        sensors::SensorManager *sensorManager = sensors::SensorManager::Instance();
        for (int i = 0; i < 4; ++i) {
            if (!footContactSensorNames_[i].empty()) {
                sensors::SensorPtr sensor = sensorManager->GetSensor(footContactSensorNames_[i]);
                if (sensor) {
                    footContactSensors_[i] = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
                    if (footContactSensors_[i]) {
                        gzerr << "[UnitreeGazeboInterface] Foot sensor " << i
                              << " found: " << footContactSensorNames_[i] << std::endl;
                    }
                }
            }
        }
    }

    // Initialize Unitree SDK2 DDS
    gzerr << "[UnitreeGazeboInterface] Initializing DDS channel factory..." << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(domainId_, interface_);

    // Create publisher for LowState
    lowstatePublisher_.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    lowstatePublisher_->InitChannel();
    gzerr << "[UnitreeGazeboInterface] LowState publisher initialized" << std::endl;

    // Create subscriber for LowCmd
    lowcmdSubscriber_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
    lowcmdSubscriber_->InitChannel(std::bind(&UnitreeGazeboInterface::LowCmdCallback, this, std::placeholders::_1), 1);
    gzerr << "[UnitreeGazeboInterface] LowCmd subscriber initialized" << std::endl;

    // Initialize low command with safe defaults
    for (int i = 0; i < 20; ++i) {
        lowCmd_.motor_cmd()[i].mode() = 0x00;  // Passive mode
        lowCmd_.motor_cmd()[i].q() = 0.0f;
        lowCmd_.motor_cmd()[i].dq() = 0.0f;
        lowCmd_.motor_cmd()[i].tau() = 0.0f;
        lowCmd_.motor_cmd()[i].kp() = 0.0f;
        lowCmd_.motor_cmd()[i].kd() = 0.0f;
    }

    // Connect to Gazebo update event
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&UnitreeGazeboInterface::OnUpdate, this));

    // Initialize timing
    lastUpdateTime_ = model_->GetWorld()->SimTime();

    // Start publish thread
    publishThread_ = std::make_unique<std::thread>(&UnitreeGazeboInterface::PublishLoop, this);

    initialized_ = true;
    gzerr << "[UnitreeGazeboInterface] Plugin loaded successfully" << std::endl;
}

void UnitreeGazeboInterface::LowCmdCallback(const void *message) {
    std::lock_guard<std::mutex> lock(lowCmdMutex_);
    lowCmd_ = *(unitree_go::msg::dds_::LowCmd_ *)message;
}

void UnitreeGazeboInterface::OnUpdate() {
    // Apply motor commands from LowCmd
    std::lock_guard<std::mutex> lock(lowCmdMutex_);

    for (int i = 0; i < numMotors_ && i < 20; ++i) {
        const auto &cmd = lowCmd_.motor_cmd()[i];

        // Only apply commands if motor is in servo mode
        if (cmd.mode() != 0x01) {
            gzerr << "[UnitreeGazeboInterface] Motor " << i << " is not in servo mode" << std::endl;
        }

        // Get current joint state
        double q_actual = joints_[i]->Position(0);
        double dq_actual = joints_[i]->GetVelocity(0);

        // Compute PD control with feedforward torque
        // tau = tau_ff + kp * (q_des - q_actual) + kd * (dq_des - dq_actual)
        double tau = cmd.tau() + cmd.kp() * (cmd.q() - q_actual) + cmd.kd() * (cmd.dq() - dq_actual);

        // Apply torque to joint
        joints_[i]->SetForce(0, tau);
    }
}

void UnitreeGazeboInterface::UpdateLowState() {
    // Update motor states
    for (int i = 0; i < numMotors_ && i < 20; ++i) {
        lowState_.motor_state()[i].q() = static_cast<float>(joints_[i]->Position(0));
        lowState_.motor_state()[i].dq() = static_cast<float>(joints_[i]->GetVelocity(0));
        lowState_.motor_state()[i].tau_est() = static_cast<float>(joints_[i]->GetForce(0));
    }

    // Update IMU state
    if (imuSensor_) {
        // Get orientation quaternion
        ignition::math::Quaterniond orientation = imuSensor_->Orientation();
        lowState_.imu_state().quaternion()[0] = static_cast<float>(orientation.W());
        lowState_.imu_state().quaternion()[1] = static_cast<float>(orientation.X());
        lowState_.imu_state().quaternion()[2] = static_cast<float>(orientation.Y());
        lowState_.imu_state().quaternion()[3] = static_cast<float>(orientation.Z());

        // Compute RPY from quaternion
        double w = orientation.W();
        double x = orientation.X();
        double y = orientation.Y();
        double z = orientation.Z();
        lowState_.imu_state().rpy()[0] = static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
        lowState_.imu_state().rpy()[1] = static_cast<float>(std::asin(2 * (w * y - z * x)));
        lowState_.imu_state().rpy()[2] = static_cast<float>(std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));

        // Get angular velocity
        ignition::math::Vector3d angularVel = imuSensor_->AngularVelocity();
        lowState_.imu_state().gyroscope()[0] = static_cast<float>(angularVel.X());
        lowState_.imu_state().gyroscope()[1] = static_cast<float>(angularVel.Y());
        lowState_.imu_state().gyroscope()[2] = static_cast<float>(angularVel.Z());

        // Get linear acceleration
        ignition::math::Vector3d linearAcc = imuSensor_->LinearAcceleration();
        lowState_.imu_state().accelerometer()[0] = static_cast<float>(linearAcc.X());
        lowState_.imu_state().accelerometer()[1] = static_cast<float>(linearAcc.Y());
        lowState_.imu_state().accelerometer()[2] = static_cast<float>(linearAcc.Z());
    } else {
        // Get IMU data from base link if no IMU sensor
        physics::LinkPtr baseLink = model_->GetLink("base");
        if (!baseLink) {
            baseLink = model_->GetLinks().front();  // Use first link as fallback
        }

        if (baseLink) {
            // Get orientation
            ignition::math::Pose3d pose = baseLink->WorldPose();
            ignition::math::Quaterniond orientation = pose.Rot();

            lowState_.imu_state().quaternion()[0] = static_cast<float>(orientation.W());
            lowState_.imu_state().quaternion()[1] = static_cast<float>(orientation.X());
            lowState_.imu_state().quaternion()[2] = static_cast<float>(orientation.Y());
            lowState_.imu_state().quaternion()[3] = static_cast<float>(orientation.Z());

            // Compute RPY
            double w = orientation.W();
            double x = orientation.X();
            double y = orientation.Y();
            double z = orientation.Z();
            lowState_.imu_state().rpy()[0] =
                static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
            lowState_.imu_state().rpy()[1] = static_cast<float>(std::asin(2 * (w * y - z * x)));
            lowState_.imu_state().rpy()[2] =
                static_cast<float>(std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));

            // Get angular velocity in world frame, transform to body frame
            ignition::math::Vector3d angularVelWorld = baseLink->WorldAngularVel();
            ignition::math::Vector3d angularVelBody = orientation.Inverse().RotateVector(angularVelWorld);
            lowState_.imu_state().gyroscope()[0] = static_cast<float>(angularVelBody.X());
            lowState_.imu_state().gyroscope()[1] = static_cast<float>(angularVelBody.Y());
            lowState_.imu_state().gyroscope()[2] = static_cast<float>(angularVelBody.Z());

            // Get linear acceleration (approximate from velocity change)
            ignition::math::Vector3d linearAccWorld = baseLink->WorldLinearAccel();
            // Add gravity to get accelerometer reading
            linearAccWorld.Z() += 9.81;
            ignition::math::Vector3d linearAccBody = orientation.Inverse().RotateVector(linearAccWorld);
            lowState_.imu_state().accelerometer()[0] = static_cast<float>(linearAccBody.X());
            lowState_.imu_state().accelerometer()[1] = static_cast<float>(linearAccBody.Y());
            lowState_.imu_state().accelerometer()[2] = static_cast<float>(linearAccBody.Z());
        }
    }

    // Update foot force sensors (if available)
    if (hasFootSensors_) {
        for (int i = 0; i < 4; ++i) {
            if (footContactSensors_[i]) {
                msgs::Contacts contacts = footContactSensors_[i]->Contacts();
                double totalForce = 0.0;
                for (int j = 0; j < contacts.contact_size(); ++j) {
                    const msgs::Contact &contact = contacts.contact(j);
                    for (int k = 0; k < contact.wrench_size(); ++k) {
                        const msgs::JointWrench &wrench = contact.wrench(k);
                        ignition::math::Vector3d force(wrench.body_1_wrench().force().x(),
                                                       wrench.body_1_wrench().force().y(),
                                                       wrench.body_1_wrench().force().z());
                        totalForce += force.Length();
                    }
                }
                lowState_.foot_force()[i] = static_cast<int16_t>(totalForce);
            } else {
                lowState_.foot_force()[i] = 0;
            }
        }
    }

    // Update tick (timestamp in milliseconds)
    common::Time simTime = model_->GetWorld()->SimTime();
    lowState_.tick() = static_cast<uint32_t>(simTime.Double() * 1000.0);
}

void UnitreeGazeboInterface::PublishLoop() {
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

GZ_REGISTER_MODEL_PLUGIN(UnitreeGazeboInterface)

}  // namespace gazebo
