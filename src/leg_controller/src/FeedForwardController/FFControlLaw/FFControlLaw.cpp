#include "FFControlLaw.h"

namespace ff_control_law
{
    FFControlLaw::FFControlLaw()
    {
        _B.setZero();
        _Su.setZero();
        _qd_dot.setZero();
    }

    Eigen::Matrix<double, M, 1> FFControlLaw::computeFFTorque(const Eigen::Matrix<double, N, N> &D,
                                                            const Eigen::Matrix<double, N, 1> &H,
                                                            const Eigen::Matrix<double, N, 1> &q,
                                                            const Eigen::Matrix<double, N, 1> &q_dot)
    {
        Eigen::Matrix<double, N, 1> qd_dot = _qd_dot;

        /*QR decomposition for constraint jacobian J*/
        Eigen::Matrix<double, K, N> Jc;
        // Jc = computeConstrainJacobian(q, q_dot);

        Eigen::HouseholderQR<Eigen::MatrixXd> qr(Jc.transpose());
        Eigen::MatrixXd Q = qr.householderQ();

        /*Compute feed forward torque*/
        Eigen::Matrix<double, M, 1> torque_ff;
        Eigen::MatrixXd Q_T = Q.transpose();
        torque_ff = (_Su * Q_T * _B).completeOrthogonalDecomposition().solve(_Su) * Q_T * (D * qd_dot + H);

        return torque_ff;
    }
}