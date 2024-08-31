#ifndef MOTOR_CONTROLLER_FSM_H
#define MOTOR_CONTROLLER_FSM_H
#include "../../ActiveObject/ActiveObject.h"
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include "../ControlLaw/ControlLaw.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#define NOT_YET 0
#define DONE 1

namespace motor_ctrl_fsm
{
    /*Finite state machine for low level control node*/
    enum MotorControllerEvent
    {
        SENSOR_SENDED_SIG = ao::USER_SIG + 1,
        ABNORMAL_STATE_SIG,
        NEW_MESSAGE_SIG
    };
    class MotorControllerFSM : public ao::ActiveObject
    {
    public:
        MotorControllerFSM();

        /**
         * Get motor torque vector
         */
        Eigen::VectorXd getMotorTorque() { return _motor_torque; }


        /**
         * Update motor state
         */
        inline void updateMotorState(const sensor_msgs::msg::JointState::SharedPtr motor_state_msg)
        {
            for (std::size_t index = 0; index < motor_state_msg->position.size(); index++)
            {
                _measured_motor_pos[index] = motor_state_msg->position[index];
            }

            for (std::size_t index = 0; index < motor_state_msg->velocity.size(); index++)
            {
                _measured_motor_vel[index] = motor_state_msg->velocity[index];
            }
        }

        /**
         * Update desired motor state
         */
        inline void updateDesiredMotorState(const sensor_msgs::msg::JointState::SharedPtr desired_motor_state_msg)
        {
            for (std::size_t index = 0; index < desired_motor_state_msg->position.size(); index++)
            {
                _desired_motor_pos[index] = desired_motor_state_msg->position[index];
            }

            for (std::size_t index = 0; index < desired_motor_state_msg->velocity.size(); index++)
            {
                _desired_motor_vel[index] = desired_motor_state_msg->velocity[index];
            }
        }

        /**
         * Update desired motor torque
         */
        inline void updateDesiredTorque(const std_msgs::msg::Float64MultiArray::SharedPtr desired_motor_state_msg)
        {
            for (std::size_t index = 0; index < desired_motor_state_msg->data.size(); index++)
            {
                _desired_torque[index] = desired_motor_state_msg->data[index];
            }
        }

        /**
         * Update parameters
         */
        inline void setKP(const rclcpp::Parameter &K_P)
        {
            _torque_control_law.setKP(K_P);
        }

        inline void setKD(const rclcpp::Parameter &K_D)
        {
            _torque_control_law.setKD(K_D);
        }

        inline void setLowerBoundTorque(const rclcpp::Parameter &LOWER_BOUND)
        {
            _torque_control_law.setLowerBoundTorque(LOWER_BOUND);
        }

        inline void setUpperBoundTorque(const rclcpp::Parameter &UPPER_BOUND)
        {
            _torque_control_law.setUpperBoundTorque(UPPER_BOUND);
        }

        private :
        /*Cache value for torque control law*/
        Eigen::VectorXd _measured_motor_pos;
        Eigen::VectorXd _measured_motor_vel;
        Eigen::VectorXd _desired_motor_pos;
        Eigen::VectorXd _desired_motor_vel;
        Eigen::VectorXd _desired_torque;
        Eigen::VectorXd _motor_torque;
        control_law::ControlLaw _torque_control_law;


        /*State machine*/
        ao::Status initial(const ao::Event *const event);
        ao::Status executing(const ao::Event *const event);
        ao::Status safety_off(const ao::Event *const event);
    };
}

#endif /*MOTOR_CONTROLLER_FSM_H*/