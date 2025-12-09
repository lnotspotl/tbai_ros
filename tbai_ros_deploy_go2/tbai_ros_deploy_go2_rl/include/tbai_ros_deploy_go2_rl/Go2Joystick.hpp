#pragma once

#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tbai_core/config/Config.hpp>
#include <tbai_deploy_go2/Go2JoystickInterface.hpp>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace reference {

class Go2Joystick : public ::tbai::reference::ReferenceVelocityGenerator, public ::tbai::go2::Go2JoystickInterface {
   public:
    Go2Joystick(ros::NodeHandle &nh) : Go2JoystickInterface(), ReferenceVelocityGenerator() {
        changeControllerPublisher_ = nh.advertise<std_msgs::String>("/anymal_d/change_controller", 10);
        TBAI_LOG_INFO(logger_, "Go2Joystick initialized");

        velocityFactorX_ = tbai::fromGlobalConfig<scalar_t>("go2_joystick/velocity_factor_x", 0.6);
        velocityFactorY_ = tbai::fromGlobalConfig<scalar_t>("go2_joystick/velocity_factor_y", 0.5);
        yawRateFactor_ = tbai::fromGlobalConfig<scalar_t>("go2_joystick/yaw_rate_factor", 0.4);

        TBAI_LOG_INFO(logger_, "Velocity factor X: {}", velocityFactorX_);
        TBAI_LOG_INFO(logger_, "Velocity factor Y: {}", velocityFactorY_);
        TBAI_LOG_INFO(logger_, "Yaw rate factor: {}", yawRateFactor_);

        twistPublisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    }

    void publishTwist() {
        geometry_msgs::Twist twist;
        twist.linear.x = gamepad.ly * velocityFactorX_;
        twist.linear.y = -gamepad.lx * velocityFactorY_;
        twist.angular.z = -gamepad.rx * yawRateFactor_;
        twistPublisher_.publish(twist);
    }

    ReferenceVelocity getReferenceVelocity(scalar_t time, scalar_t dt) {
        ReferenceVelocity reference;
        reference.velocity_x = gamepad.ly * velocityFactorX_;
        reference.velocity_y = -gamepad.lx * velocityFactorY_;
        reference.yaw_rate = -gamepad.rx * yawRateFactor_;
        return reference;
    }

    void onPressA() override {
        std_msgs::String msg;
        msg.data = "SIT";
        changeControllerPublisher_.publish(msg);
        TBAI_LOG_WARN(logger_, "A pressed: Changing controller to {}", msg.data);
    }

    void onPressB() override {
        std_msgs::String msg;
        msg.data = "BOB";
        changeControllerPublisher_.publish(msg);
        TBAI_LOG_WARN(logger_, "B pressed: Changing controller to {}", msg.data);
    }

    void onPressX() override {
        std_msgs::String msg;
        msg.data = "STAND";
        changeControllerPublisher_.publish(msg);
        TBAI_LOG_WARN(logger_, "X pressed: Changing controller to {}", msg.data);
    }

    void onPressY() override {
        std_msgs::String msg;
        msg.data = "NP3O";
        changeControllerPublisher_.publish(msg);
        TBAI_LOG_WARN(logger_, "Y pressed: Changing controller to {}", msg.data);
    }

   private:
    ros::Publisher changeControllerPublisher_;
    ros::NodeHandle nh_;
    ros::Publisher twistPublisher_;

    scalar_t yawRateFactor_ = 0.4;
    scalar_t velocityFactorX_ = 0.6;
    scalar_t velocityFactorY_ = 0.5;
};

std::unique_ptr<Go2Joystick> getGo2JoystickUnique(ros::NodeHandle &nh) {
    return std::make_unique<Go2Joystick>(nh);
}

std::shared_ptr<Go2Joystick> getGo2JoystickShared(ros::NodeHandle &nh) {
    return std::make_shared<Go2Joystick>(nh);
}

}  // namespace reference
}  // namespace tbai
