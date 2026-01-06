// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "tbai_ros_gazebo/StatePublisher.hpp"

#include <functional>
#include <string>

#include <Eigen/Geometry>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_ros_msgs/RbdState.h>

namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    TBAI_GLOBAL_LOG_INFO("Loading StatePublisher plugin");

    bool enabled = tbai::fromGlobalConfig<bool>("gazebo/ground_truth_state_publisher/enabled");
    if (!enabled) {
        TBAI_GLOBAL_LOG_INFO("Ground truth state publisher disabled.");
        return;
    }

    fixedBase_ = tbai::fromGlobalConfig<bool>("gazebo/ground_truth_state_publisher/fixed_base", false);

    // set Gazebo callback function
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&StatePublisher::OnUpdate, this));

    this->robot_ = robot;

    ros::NodeHandle nh;
    auto stateTopic = tbai::fromGlobalConfig<std::string>("state_topic");
    statePublisher_ = nh.advertise<tbai_ros_msgs::RbdState>(stateTopic, 2);

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

    rate_ = tbai::fromGlobalConfig<double>("gazebo/ground_truth_state_publisher/update_rate");
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

    TBAI_GLOBAL_LOG_INFO("Loaded StatePublisher plugin");
}  // namespace gazebo

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StatePublisher::OnUpdate() {
    // Get current time
    const common::Time currentTime = robot_->GetWorld()->SimTime();
    const double dt = (currentTime - lastSimTime_).Double();

    // Check if update is needed
    if (dt < period_) {
        return;
    }

    // Put everything into an RbdState message
    tbai_ros_msgs::RbdState message;  // TODO(lnotspotl): Room for optimization here

    // Resize arrays upfront
    const size_t rbdStateSize = (fixedBase_ ? 0 : 12) + 2 * joints_.size();
    message.rbd_state.resize(rbdStateSize);
    message.contact_flags.resize(contactFlags_.size());

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
            firstUpdate_ = false;
        }

        // Base angular velocity in base frame
        const Eigen::Vector3d angularVelocityWorld = tbai::mat2aa(R_world_base * lastOrientationBase2World_) / dt;
        const Eigen::Vector3d angularVelocityBase = R_base_world * angularVelocityWorld;

        // Base linear velocity in base frame
        const Eigen::Vector3d linearVelocityWorld = (basePosition - lastPositionBase_) / dt;
        const Eigen::Vector3d linearVelocityBase = R_base_world * linearVelocityWorld;

        message.rbd_state[0] = rpy[0];
        message.rbd_state[1] = rpy[1];
        message.rbd_state[2] = rpy[2];

        // Base position
        message.rbd_state[3] = basePosition[0];
        message.rbd_state[4] = basePosition[1];
        message.rbd_state[5] = basePosition[2];

        // Base angular velocity
        message.rbd_state[6] = angularVelocityBase[0];
        message.rbd_state[7] = angularVelocityBase[1];
        message.rbd_state[8] = angularVelocityBase[2];

        // Base linear velocity
        message.rbd_state[9] = linearVelocityBase[0];
        message.rbd_state[10] = linearVelocityBase[1];
        message.rbd_state[11] = linearVelocityBase[2];

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

    // Joint positions
    const size_t offsetAngles = (fixedBase_ ? 0 : 12);
    for (int i = 0; i < jointAngles.size(); ++i) {
        message.rbd_state[offsetAngles + i] = jointAngles[i];
    }

    // Joint velocities
    const size_t offsetVelocities = (fixedBase_ ? 0 : 12) + jointAngles.size();
    for (int i = 0; i < jointVelocities.size(); ++i) {
        message.rbd_state[offsetVelocities + i] = jointVelocities[i];
    }

    // Observation time
    message.stamp = ros::Time::now();

    // Contact flags
    std::copy(contactFlags_.begin(), contactFlags_.end(), message.contact_flags.begin());

    // Publish message
    statePublisher_.publish(message);

    lastSimTime_ = currentTime;
}

GZ_REGISTER_MODEL_PLUGIN(StatePublisher);

}  // namespace gazebo
