#include "MotorControllerFsm.h"

namespace motor_ctrl_fsm
{
    /*Finite state machine for low level control node*/
    MotorControllerFSM::MotorControllerFSM():
        ao::ActiveObject::ActiveObject()
    {
        this->_state = (StateHandler)&MotorControllerFSM::initial;
        this->_torque_control_law = control_law::ControlLaw();

        this->_motor_torque = Eigen::VectorXd::Zero(10);
        this->_desired_torque = Eigen::VectorXd::Zero(10);
        this->_measured_motor_pos = Eigen::VectorXd::Zero(10);
        this->_measured_motor_vel = Eigen::VectorXd::Zero(10);
        this->_desired_motor_pos = Eigen::VectorXd::Zero(10);
        this->_desired_motor_vel = Eigen::VectorXd::Zero(10);
    }

    /*Initial state*/
    ao::Status MotorControllerFSM::initial(const ao::Event * const event)
    {   
        ao::Event temp(event->_signal);
        ao::Status status = ao::TRAN_STATUS;
        this->_state = (StateHandler)&MotorControllerFSM::executing;
        return status;
    }

    /*Executing state*/
    ao::Status MotorControllerFSM::executing(const ao::Event * const event)
    {
        ao::Status status;
        switch (event->_signal)
        {
            case ao::ENTRY_SIG:
            {
                status = ao::HANDLED_STATUS;
                break;
            }

            case ao::EXIT_SIG:
            {
                status = ao::HANDLED_STATUS;
                break;
            }

            case MotorControllerEvent::ABNORMAL_STATE_SIG:
            {
                this->_state = (StateHandler)&MotorControllerFSM::safety_off;
                status = ao::TRAN_STATUS;
                break;
            }

            case ao::DEFAULT_SIG:
            {
                /*Check Not A Number NaN*/
                if(!_measured_motor_pos.array().allFinite()) _measured_motor_pos.setZero();
                if(!_measured_motor_vel.array().allFinite()) _measured_motor_vel.setZero();
                if(!_desired_torque.array().allFinite()) _desired_torque.setZero();

                /*Normal case*/
                _motor_torque = _torque_control_law.updateTorque(_measured_motor_pos, _measured_motor_vel, _desired_torque);
                    
                status = ao::HANDLED_STATUS;
                break;
            }

        }

        return status;
    }

    /*Abnormal state - shut down motor immediately*/
    ao::Status MotorControllerFSM::safety_off(const ao::Event *const event)
    {
        ao::Status status;
        switch (event->_signal)
        {
            case ao::ENTRY_SIG:
            {
                this->_motor_torque = Eigen::VectorXd::Zero(10);
                status = ao::HANDLED_STATUS;
                break;
            }

            case ao::EXIT_SIG:
            {
                status = ao::HANDLED_STATUS;
                break;
            }

            case ao::DEFAULT_SIG:
            {
                status = ao::HANDLED_STATUS;
                break;
            }

        }

        return status;
    }
}
