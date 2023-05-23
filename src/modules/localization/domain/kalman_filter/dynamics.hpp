#ifndef KALMAN_FILTER__DYNAMICS_H
#define KALMAN_FILTER__DYNAMICS_H

#include<eigen3/Eigen/Dense>

namespace kalman_filter {
    class Dynamics{
        public:

        /** @brief sets the matrix values depending on the dynamic model
         * 
         * @param[in] x The current state
         * @param[in] F The state transition matrix
         * @param[in] P The covariance matrix of the current state
         * @param[in] Q The process noise matrix
         * @param[in] H The observation matrix
         * @param[in] R The covariance matrix of the measurement
        */
        void set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, 
                                Eigen::MatrixXd &Q, Eigen::MatrixXd &H, Eigen::MatrixXd &R);
    };
}



#endif