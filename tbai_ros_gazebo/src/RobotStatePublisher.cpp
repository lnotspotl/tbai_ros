// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_gazebo/RobotStatePublisher.hpp"

#include <functional>
#include <string>

#include <Eigen/Geometry>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_ros_msgs/RobotState.h>

namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RobotStatePublisher::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    TBAI_GLOBAL_LOG_INFO("Loading RobotStatePublisher plugin");

    bool enabled = tbai::fromGlobalConfig<bool>("gazebo/muse_state_publisher/enabled");
    if (!enabled) {
        TBAI_GLOBAL_LOG_INFO("Muse state publisher disabled.");
        return;
    }

    fixedBase_ = tbai::fromGlobalConfig<bool>("gazebo/muse_state_publisher/fixed_base", false);

    // set Gazebo callback function
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RobotStatePublisher::OnUpdate, this));

    this->robot_ = robot;

    ros::NodeHandle nh;
    auto stateTopic = tbai::fromGlobalConfig<std::string>("state_topic");
    statePublisher_ = nh.advertise<tbai_ros_msgs::RobotState>(stateTopic, 2);

    if (!fixedBase_) {
        auto base = tbai::fromGlobalConfig<std::string>("base_name");
        baseLinkPtr_ = robot->GetChildLink(base);
        bool found = baseLinkPtr_ != nullptr;
        TBAI_GLOBAL_LOG_INFO("Base link {} found: {}", base, found);
    }

    // get joints; ignore 'universe' and 'root_joint'
    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    for (int i = 0; i < jointNames.size(); ++i) {
        joints_.push_back(robot->GetJoint(jointNames[i]));
        bool found = joints_.back() != nullptr;
        TBAI_GLOBAL_LOG_INFO("Joint {} found: {}", jointNames[i], found);
    }

    // initialize last publish time
    lastSimTime_ = robot->GetWorld()->SimTime();

    rate_ = tbai::fromGlobalConfig<double>("gazebo/muse_state_publisher/update_rate");
    period_ = 1.0 / rate_;

    // get contact topics
    auto contactTopics = tbai::fromGlobalConfig<std::vector<std::string>>("contact_topics", std::vector<std::string>());
    contactFlags_.resize(contactTopics.size(), false);
    contactSubscribers_.resize(contactTopics.size());
    for (int i = 0; i < contactTopics.size(); ++i) {
        TBAI_GLOBAL_LOG_INFO("Subscribing to contact topic: {}", contactTopics[i]);
        auto callback = [this, i](const std_msgs::Bool::ConstPtr &msg) { contactFlags_[i] = msg->data; };
        contactSubscribers_[i] = nh.subscribe<std_msgs::Bool>(contactTopics[i], 1, callback);
    }

    TBAI_GLOBAL_LOG_INFO("Loaded RobotStatePublisher plugin");
}  // namespace gazebo

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void RobotStatePublisher::OnUpdate() {
    // Get current time
    const common::Time currentTime = robot_->GetWorld()->SimTime();
    const double dt = (currentTime - lastSimTime_).Double();

    // Check if update is needed
    if (dt < period_) {
        return;
    }

    // Put everything into an RbdState message
    tbai_ros_msgs::RobotState message;  // TODO(lnotspotl): Room for optimization here

    if (!fixedBase_) {
        // Unpack base pose
        const ignition::math::Pose3d &basePoseIgn = baseLinkPtr_->WorldPose();
        const auto &basePositionIgn = basePoseIgn.Pos();
        const auto &baseOrientationIgn = basePoseIgn.Rot();

        // Base orientation - Euler zyx
        const Eigen::Quaternion<double> baseQuaternion(baseOrientationIgn.W(), baseOrientationIgn.X(),
                                                       baseOrientationIgn.Y(), baseOrientationIgn.Z());
        const tbai::matrix3_t R_world_base = baseQuaternion.toRotationMatrix();
        const tbai::matrix3_t R_base_world = R_world_base.transpose();
        const tbai::vector_t rpy = tbai::mat2oc2rpy(R_world_base, lastYaw_);
        lastYaw_ = rpy[2];

        // Base position in world frame
        const Eigen::Vector3d basePosition(basePositionIgn.X(), basePositionIgn.Y(), basePositionIgn.Z());

        if (firstUpdate_) {
            lastOrientationBase2World_ = R_base_world;
            lastPositionBase_ = basePosition;
            lastVelocityBase_ = tbai::vector3_t::Zero();
            firstUpdate_ = false;
        }

        // Base angular velocity in base frame
        const Eigen::Vector3d angularVelocityWorld = tbai::mat2aa(R_world_base * lastOrientationBase2World_) / dt;
        const Eigen::Vector3d angularVelocityBase = R_base_world * angularVelocityWorld;

        // Base linear velocity in base frame
        const Eigen::Vector3d linearVelocityWorld = (basePosition - lastPositionBase_) / dt;
        const Eigen::Vector3d linearVelocityBase = R_base_world * linearVelocityWorld;

        // Base linear acceleration in base frame
        Eigen::Vector3d linearAccelerationWorld = (linearVelocityWorld - lastVelocityBase_) / dt;
        Eigen::Vector3d linearAccelerationBase =
            R_base_world * (linearAccelerationWorld + tbai::vector3_t::UnitZ() * 9.81);
        lastVelocityBase_ = linearVelocityWorld;

        // Base orientation - Euler zyx as {roll, pitch, yaw}
        message.orientation_xyzw[0] = baseOrientationIgn.X();
        message.orientation_xyzw[1] = baseOrientationIgn.Y();
        message.orientation_xyzw[2] = baseOrientationIgn.Z();
        message.orientation_xyzw[3] = baseOrientationIgn.W();

        // Base linear acceleration
        message.lin_acc[0] = linearAccelerationBase[0];
        message.lin_acc[1] = linearAccelerationBase[1];
        message.lin_acc[2] = linearAccelerationBase[2];

        // Base angular velocity
        message.ang_vel[0] = angularVelocityBase[0];
        message.ang_vel[1] = angularVelocityBase[1];
        message.ang_vel[2] = angularVelocityBase[2];

        lastOrientationBase2World_ = R_base_world;
        lastPositionBase_ = basePosition;
    }

    // Joint angles
    std::vector<double> jointAngles(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointAngles[i] = joints_[i]->Position(0);
    }

    // Joint velocities
    if (lastJointAngles_.size() != joints_.size()) {
        lastJointAngles_.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); ++i) {
            lastJointAngles_[i] = jointAngles[i];
        }
    }

    // Get joint velocities
    std::vector<double> jointVelocities(joints_.size());
    for (int i = 0; i < joints_.size(); ++i) {
        jointVelocities[i] = (jointAngles[i] - lastJointAngles_[i]) / dt;
        lastJointAngles_[i] = jointAngles[i];
    }

    // Resize arrays
    message.joint_angles.resize(jointAngles.size());
    message.joint_velocities.resize(jointVelocities.size());
    message.contact_flags.resize(contactFlags_.size());

    // Joint positions
    for (int i = 0; i < jointAngles.size(); ++i) {
        message.joint_angles[i] = jointAngles[i];
    }

    // Joint velocities
    for (int i = 0; i < jointVelocities.size(); ++i) {
        message.joint_velocities[i] = jointVelocities[i];
    }

    // Observation time
    message.stamp = ros::Time::now();

    // Contact flags
    std::copy(contactFlags_.begin(), contactFlags_.end(), message.contact_flags.begin());

    // Publish message
    statePublisher_.publish(message);

    lastSimTime_ = currentTime;
}

GZ_REGISTER_MODEL_PLUGIN(RobotStatePublisher);

}  // namespace gazebo
