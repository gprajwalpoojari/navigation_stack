#ifndef SENSOR_FUSION__EXTENDED_KALMAN_FILTER__DYNAMICS_HPP
#define SENSOR_FUSION__EXTENDED_KALMAN_FILTER__DYNAMICS_HPP

#include<eigen3/Eigen/Dense>

namespace sensor_fusion::extended_kalman_filter {
    class Dynamics{
        public:

        /** @brief sets the matrix values depending on the dynamic model
         * 
         * @param[in] x The current state
         * @param[in] F The state transition matrix
         * @param[in] P The covariance matrix of the current state
         * @param[in] Q The process noise matrix
        */
        void set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, 
                                Eigen::MatrixXd &Q);
    };
}



#endif