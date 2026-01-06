#pragma once

#include <vector>

#include <Eigen/Dense>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tbai_core/Types.hpp>

namespace gazebo {
class RobotStatePublisher : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    event::ConnectionPtr updateConnection_;

    /** RbdState message publisher */
    ros::Publisher statePublisher_;

    /** Robot gazebo model */
    physics::ModelPtr robot_;

    /** Base link */
    physics::LinkPtr baseLinkPtr_;

    std::vector<physics::JointPtr> joints_;

    /** State publish rate */
    double rate_;
    double period_;

    bool firstUpdate_ = true;

    double lastYaw_ = 0.0;

    std::vector<bool> contactFlags_;
    std::vector<ros::Subscriber> contactSubscribers_;

    bool fixedBase_ = false;

    // last yaw angle
    std::vector<tbai::scalar_t> lastJointAngles_;
    tbai::matrix3_t lastOrientationBase2World_;
    tbai::vector3_t lastPositionBase_;
    tbai::vector3_t lastVelocityBase_;
    common::Time lastSimTime_;
};

}  // namespace gazebo
