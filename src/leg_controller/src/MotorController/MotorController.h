#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "MotorControllerFsm/MotorControllerFsm.h"
#include "MotorConfig.h"



namespace motor_ctrl
{
    /*Low level control node: calculate torque*/
    class MotorControllerNode: public rclcpp::Node
    {
        public:
            MotorControllerNode();

        private:
            /*FInite state machine*/
            motor_ctrl_fsm::MotorControllerFSM _fsm;

            /*Timer*/
            rclcpp::TimerBase::SharedPtr _timer;

            /*Motor configurations hub*/
            std::array<motor_config::MotorConfig, 10> _motor_configs = {
                motor_config::MotorConfig("/model/biped_robot/joint/R_hip_joint/cmd_force", 0),
                motor_config::MotorConfig("/model/biped_robot/joint/R_hip2_joint/cmd_force", 1),
                motor_config::MotorConfig("/model/biped_robot/joint/R_thigh_joint/cmd_force", 2),
                motor_config::MotorConfig("/model/biped_robot/joint/R_calf_joint/cmd_force", 3),
                motor_config::MotorConfig("/model/biped_robot/joint/R_toe_joint/cmd_force", 4),
                motor_config::MotorConfig("/model/biped_robot/joint/L_hip_joint/cmd_force", 5),
                motor_config::MotorConfig("/model/biped_robot/joint/L_hip2_joint/cmd_force", 6),
                motor_config::MotorConfig("/model/biped_robot/joint/L_thigh_joint/cmd_force", 7),
                motor_config::MotorConfig("/model/biped_robot/joint/L_calf_joint/cmd_force", 8),
                motor_config::MotorConfig("/model/biped_robot/joint/L_toe_joint/cmd_force", 9)};

            /*Motor publisher hub*/
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 10> _motor_publisher_hub;

            /*Motor states subsriber*/
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _motor_state_sub;

            /*Desired motor states subsriber*/
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _desired_motor_state_sub;

            /*Desired motor torque subsriber*/
            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _desired_motor_torque_sub;

            /*Parameter callback handler*/
            OnSetParametersCallbackHandle::SharedPtr _parameter_handler;

            /*Node parameters*/
            rclcpp::Parameter _K_P;
            rclcpp::Parameter _K_D;
            rclcpp::Parameter _UPPER_BOUND_TORQUE;
            rclcpp::Parameter _LOWER_BOUND_TORQUE;

            /**
             * Handle Timer Events, creating sampling time
             */
            void handleTimerEvt();

            /**
             * Cached derised motor state on desired motor state msg update event
             * @param motor_state_msg pointer point to msg
             */
            void handleDesiredMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg);

            /**
             * Cached motor state on motor state msg update event
             * @param motor_state_msg pointer point to msg
             */
            void handleMotorStateAvailableEvt(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg);

            /**
             * Cached desired motor torque on desired torque msg update event
             * @param desired_motor_torque_msg pointer point to msg
             */
            void handleDesiredTorqueAvailableEvt(const std_msgs::msg::Float64MultiArray::SharedPtr desired_motor_torque_msg);

            /**
             * Cached parameter change
             * @param parameters 
             */
            rcl_interfaces::msg::SetParametersResult handleParameterChangeEvt(const std::vector<rclcpp::Parameter> &parameters);

            /**
             * Publish motor torque to motor interface
             * @param motor_torques vector of motor torque computed from control law.
             */
            void
            publishMotorInterfaces(const Eigen::VectorXd &motor_torques);
    };
}

#endif /* MOTOR_CONTROLLER_H */