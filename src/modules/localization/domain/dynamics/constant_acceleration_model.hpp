#ifndef LOCALIZATION__DYNAMICS__DYNAMICS_HPP
#define LOCALIZATION__DYNAMICS__DYNAMICS_HPP

#include<eigen3/Eigen/Dense>

namespace localization::dynamics {
    /**
     * @brief A constant acceleleration model based on particle physics
     * 
     */
    class ConstantAccelerationModel{
        public:
        /**
         * @brief Construct a new Constant Acceleration Model object
         * 
         * @param[in] state_covariance_x 
         * @param[in] state_covariance_y 
         * @param[in] state_covariance_theta 
         * @param[in] process_noise_x 
         * @param[in] process_noise_y 
         * @param[in] process_noise_theta 
         */
        ConstantAccelerationModel(double process_noise_x=5, double process_noise_y=5, double process_noise_theta=5);

        /** @brief sets the matrix values depending on the dynamic model
         * 
         * @param[in] x The current state
         * @param[in] F The state transition matrix
         * @param[in] P The covariance matrix of the current state
         * @param[in] Q The process noise matrix
        */
        void set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, 
                                Eigen::MatrixXd &Q);


        /**
         * @brief       Updates the Process noise matrix
         * 
         * @param[in] Q     The process noise matrix
         * @param[in] dt    The time step
         */
        void update_process_noise_matrix(Eigen::MatrixXd& Q, double dt);


        /**
         * @brief       Updates the state transition matrix
         * 
         * @param[in] F     The state transition matrix
         * @param[in] dt    the time step
         */
        void update_state_matrix(Eigen::MatrixXd& F, double dt);

        private:
            const double noise_ax;    // acceleration process noise in x
            const double noise_ay;    // acceleration process noise in y
            const double noise_atheta;// acceleration process noise in theta
    };
}

#endif