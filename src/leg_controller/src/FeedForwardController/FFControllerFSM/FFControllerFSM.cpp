#include "FFControllerFSM.h"

namespace ffc_fsm
{
    FFControllerFsm::FFControllerFsm():
        ao::ActiveObject::ActiveObject()
    {
        _H.setZero();
        _D.setIdentity();
        _q.setZero();
        _q_dot.setZero();
    }

    ao::Status FFControllerFsm::initial(const ao::Event *const event)
    {
        ao::Event temp(event->_signal);
        ao::Status status = ao::TRAN_STATUS;
        this->_state = (StateHandler)&FFControllerFsm::executing;
        return status;
    }

    /*Executing state*/
    ao::Status FFControllerFsm::executing(const ao::Event *const event)
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


            case ao::DEFAULT_SIG:
            {
                Eigen::Matrix<double, M, 1> torque_ff;
                Eigen::Matrix<double, N, N> D = _D;
                Eigen::Matrix<double, N, 1> H = _H;
                Eigen::Matrix<double, N, 1> q = _q;
                Eigen::Matrix<double, N, 1> q_dot = _q_dot;

                torque_ff = _control_law.computeFFTorque(D, H, q, q_dot);

                /*Publish computed ff torque*/
                std_msgs::msg::Float64MultiArray torque_msg;
                for(uint8_t index = 0; index < M; index++) torque_msg.data[index] = torque_ff(index);
                _desired_torque_pub->publish(torque_msg);

                status = ao::HANDLED_STATUS;
                break;
            }
        }

        return status;
    }
}