#include "ControlLaw.h"

namespace control_law
{
    ControlLaw::ControlLaw()
    {
        K_P = Eigen::VectorXd::Ones(3);
        K_D = Eigen::VectorXd::Ones(3);
        LOWER_BOUND_TORQUE = Eigen::VectorXd::Ones(10) * (-1000.0);
        UPPER_BOUND_TORQUE = Eigen::VectorXd::Ones(10) * (2000.0);
    }

    Eigen::VectorXd ControlLaw::updateTorque(const Eigen::VectorXd &q,
                                             const Eigen::VectorXd &q_dot,
                                             const Eigen::VectorXd &torque_ff)
    {
        /*Compute output jacobian matrix respect to joint variable*/
        Eigen::Matrix<double, M, M> Jm;
        // e = this->computeOutputJacobian(q, q_dot);

        /*Compute actual output and desired output*/
        Eigen::VectorXd torque_feedback;
        Eigen::VectorXd yd, yd_dot;
        Eigen::VectorXd ya, ya_dot;
        Eigen::VectorXd pd_term;

        // yd = this->computeActualOutput(q, q_dot);
        // yd_dot = this->computeActualOutputDot(q, q_dot);
        // ya = this->computeDesiredOutput(q, q_dot);
        // ya_dot = this->computeDesiredOutputDot(q, q_dot);

        /*Compute feedback torque*/
        pd_term = this->K_P.cwiseProduct(ya - yd) + this->K_D.cwiseProduct(ya_dot - yd_dot);
        torque_feedback = -Jm.completeOrthogonalDecomposition().solve(pd_term);
        if (!torque_feedback.array().allFinite()) torque_feedback.setZero(10);

        /*Compute torque command*/
        Eigen::VectorXd torque_command = torque_ff + torque_feedback;
        torque_command = torque_command.cwiseMax(UPPER_BOUND_TORQUE).cwiseMin(LOWER_BOUND_TORQUE);
        return torque_command;
    }




    // Eigen::Matrix<double, M, 1> ControlLaw::computeFeedForward( const Eigen::Matrix<double, N, N> &D,
    //                                                             const Eigen::Matrix<double, N, 1> &H,
    //                                                             const Eigen::Matrix<double, K, N> &J )
    // {
    //     Eigen::Matrix<double, N, 1> setpoint_q_ddot = this->_setpoint_q_ddot;
    //     Eigen::Matrix<double, N, M> B = this->_B;
    //     Eigen::Matrix<double, N - K, N> Su = this->_Su;
        
    //     /*QR decomposition for constraint jacobian J*/
    //     Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose());
    //     Eigen::MatrixXd Q = qr.householderQ();

    //     /*Compute feed forward torque*/
    //     Eigen::Matrix<double, M, 1> torque_ff;
    //     Eigen::MatrixXd Q_T = Q.transpose();
    //     torque_ff = (Su * Q_T * B).completeOrthogonalDecomposition().solve(Su) * Q_T * (D * setpoint_q_ddot + H);

    //     this->_torque_ff = torque_ff; 
    //     return torque_ff;
    // }
}