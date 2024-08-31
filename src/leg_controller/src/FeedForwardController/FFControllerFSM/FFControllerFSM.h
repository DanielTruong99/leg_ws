#ifndef FF_CONTROLLER_FSM_H
#define FF_CONTROLLER_FSM_H

#pragma once

/*Client library*/
#include <iostream>
#include <rclcpp/rclcpp.hpp>

/*Topic type*/
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <leg_msgs/msg/estimated_robot_parameter.hpp>
#include <leg_msgs/msg/robot_operation_state.hpp>

/*User library*/
#include "../../ActiveObject/ActiveObject.h"
#include "../FFControlLaw/FFControlLaw.h"
namespace ff_controller
{
    class FeedForwardControllerNode; // Forward declaration
}

/*Third party library*/
#include <eigen3/Eigen/Dense>

namespace ffc_fsm
{
    enum FFControllerEvent
    {
        ABNORMAL_STATE_SIG = ao::USER_SIG + 1,
        NORMAL_STATE_SIG,
    };

    class FFControllerFsm : public ao::ActiveObject
    {
        public:
            ff_controller::FeedForwardControllerNode *_node_ptr;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _desired_torque_pub;

            FFControllerFsm();

            inline void updateMotorState(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg)
            {
                for (std::size_t index = 0; index < motor_state_msg->position.size(); index++)
                {
                    _q(index) = motor_state_msg->position[index];
                }

                for (std::size_t index = 0; index < motor_state_msg->velocity.size(); index++)
                {
                    _q_dot(index) = motor_state_msg->velocity[index];
                }
            }

            inline void updateRobotParameter(const leg_msgs::msg::EstimatedRobotParameter::SharedPtr robot_parameter_msg)
            {
                uint32_t num_rows = robot_parameter_msg->row_d; 
                uint32_t num_cols = robot_parameter_msg->col_d;
                for (uint32_t row_index = 0; row_index < num_rows; row_index++)
                {
                    for (uint32_t col_index = 0; col_index < num_cols; col_index++)
                    {
                        _D(row_index, col_index) = robot_parameter_msg->data_matrix_d[row_index * num_cols + col_index];
                    }

                    _H(row_index) = robot_parameter_msg->data_vector_h[row_index];
                }
            }

            inline void updateDesiredMotorState(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg)
            {
                for (std::size_t index = 0; index < desired_motor_state_msg->position.size(); index++)
                {
                    _qd(index) = desired_motor_state_msg->position[index];
                }

                for (std::size_t index = 0; index < desired_motor_state_msg->velocity.size(); index++)
                {
                    _qd_dot(index) = desired_motor_state_msg->velocity[index];
                }
            }

        private :
            #define N 16
            #define M 10

            Eigen::Matrix<double, N, 1> _qd;
            Eigen::Matrix<double, N, 1> _qd_dot;
            Eigen::Matrix<double, N, 1> _q;
            Eigen::Matrix<double, N, 1> _q_dot;
            Eigen::Matrix<double, N, N> _D;
            Eigen::Matrix<double, N, 1> _H;
            ff_control_law::FFControlLaw _control_law;

            ao::Status initial(const ao::Event *const event);
            ao::Status executing(const ao::Event *const event);
    };
}

#endif /*FF_CONTROLLER_FSM_H*/