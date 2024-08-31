#ifndef FEED_FORWAD_CONTROLLER_H
#define FEED_FORWAD_CONTROLLER_H

#pragma once

/*Client library*/
#include <chrono>
#include <rclcpp/rclcpp.hpp>

/*Topic type*/
#include <std_msgs/msg/float64_multi_array.hpp>
#include <leg_msgs/msg/estimated_robot_parameter.hpp>
#include <leg_msgs/msg/robot_operation_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/*User library*/
#include "FFControllerFSM/FFControllerFSM.h"

/*Third party library*/
#include <eigen3/Eigen/Dense>

/***/
namespace ff_controller
{
    class FeedForwardControllerNode : public rclcpp::Node
    {
        public:
            FeedForwardControllerNode();

        private:
            rclcpp::TimerBase::SharedPtr _timer;
            ffc_fsm::FFControllerFsm _fsm;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _desired_torque_pub;
            rclcpp::Subscription<leg_msgs::msg::EstimatedRobotParameter>::SharedPtr _estimated_robot_parameter_sub;
            rclcpp::Subscription<leg_msgs::msg::RobotOperationState>::SharedPtr _robot_operation_state_sub;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _motor_state_sub;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _desired_motor_state_sub;

            void handleTimerEvt();
            void handleMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg);
            void handleDesiredMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg);
            void handleRobotParameterAvailableEvt(const leg_msgs::msg::EstimatedRobotParameter::SharedPtr robot_parameter_msg);
            void handleRobotStateAvailableEvt(const leg_msgs::msg::RobotOperationState::SharedPtr robot_state_msg);
    };
}
#endif /*FEED_FORWAD_CONTROLLER_H*/