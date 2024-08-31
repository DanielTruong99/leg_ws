#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace control_law
{
    class ControlLaw
    {
        public:
            /*Number of coordinate systems*/
            #define N 16

            /*Number of constraints*/
            #define K 6

            /*Number of actuators*/
            #define M 10

            ControlLaw();


            
            /**
             * Run one control loop cycle calculation
             * @param D estimation of the current inertial matrix
             * @param H estimation of the current coriolis and gravity
             * @param Jc estimation of the current contact jacobian matrix
             * @param ya estimate of the current output
             * @return [N.m] command torque sended to motor driver
             */
            Eigen::VectorXd updateTorque(const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &q_dot,
                                         const Eigen::VectorXd &torque_ff);

            inline void setKP(const rclcpp::Parameter &KP)
            {
                std::vector<double> param_value; param_value = KP.as_double_array();
                for (std::size_t index = 0; index < param_value.size(); index++)
                {
                    K_P(index) = param_value[index];
                }
            }

            inline void setKD(const rclcpp::Parameter &KD)
            {
                std::vector<double> param_value;
                param_value = KD.as_double_array();
                for (std::size_t index = 0; index < param_value.size(); index++)
                {
                    K_D(index) = param_value[index];
                }
            }

            inline void setLowerBoundTorque(const rclcpp::Parameter &LOWER_BOUND)
            {
                std::vector<double> param_value;
                param_value = LOWER_BOUND.as_double_array();
                for (std::size_t index = 0; index < param_value.size(); index++)
                {
                    LOWER_BOUND_TORQUE(index) = param_value[index];
                }
            }

            inline void setUpperBoundTorque(const rclcpp::Parameter &UPPER_BOUND)
            {
                std::vector<double> param_value;
                param_value = UPPER_BOUND.as_double_array();
                for (std::size_t index = 0; index < param_value.size(); index++)
                {
                    UPPER_BOUND_TORQUE(index) = param_value[index];
                }
            }

        private:
            /*Selection matrix*/
            Eigen::Matrix<double, N - K, N> _Su; 

            /*Actuation matrix*/
            Eigen::Matrix<double, N, M> _B;            

            /*Parameters*/
            Eigen::VectorXd K_P;
            Eigen::VectorXd K_D;
            Eigen::VectorXd UPPER_BOUND_TORQUE;
            Eigen::VectorXd LOWER_BOUND_TORQUE;

            // /**
            //  * Run feed forward torque
            //  * @param D estimation of the current inertial matrix
            //  * @param H estimation of the current coriolis and gravity
            //  * @param J estimation of the current constraint jacobian matrix
            //  * @return [N.m] command torque sended to motor driver
            //  */
            // Eigen::Matrix<double, M, 1> computeFeedForward( const Eigen::Matrix<double, N, N> &D,
            //                                                 const Eigen::Matrix<double, N, 1> &H,
            //                                                 const Eigen::Matrix<double, K, N> &J );
    };
}

#endif /*CONTROL_LAW_H*/