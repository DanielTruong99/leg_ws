#ifndef FF_CONTROL_LAW_H
#define FF_CONTROL_LAW_H

/*Third party library*/
#include <eigen3/Eigen/Dense>

namespace ff_control_law
{
    class FFControlLaw
    {
        public:
            /*Number of coordinate systems*/
            #define N 16

            /*Number of constraints*/
            #define K 6

            /*Number of actuators*/
            #define M 10


            FFControlLaw();

            Eigen::Matrix<double, M, 1> computeFFTorque(const Eigen::Matrix<double, N, N> &D,
                                                        const Eigen::Matrix<double, N, 1> &H,
                                                        const Eigen::Matrix<double, N, 1> &q,
                                                        const Eigen::Matrix<double, N, 1> &q_dot);
        private:
            Eigen::Matrix<double, N, M> _B;
            Eigen::Matrix<double, N - K, N> _Su;
            Eigen::Matrix<double, N, 1> _qd_dot;
    };
}

#endif /* FF_CONTROL_LAW_H */