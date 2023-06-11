#ifndef LOCALIZATION__DYNAMICS__DYNAMICS_HPP
#define LOCALIZATION__DYNAMICS__DYNAMICS_HPP

#include<eigen3/Eigen/Dense>
#include <sensor_datastructures/odom.hpp>
#include <sensor_datastructures/imu.hpp>


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
         * @param[in] process_noise_ax      Acceleration process noise in x
         * @param[in] process_noise_ay      Acceleration process noise in y
         * @param[in] process_noise_atheta  Acceleration process noise in theta
         */
        ConstantAccelerationModel(double process_noise_ax=5, double process_noise_ay=5, double process_noise_atheta=5);

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

        /**
         * @brief Get the observation matrix object
         * 
         * @param odom_data         The odom data
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_observation_matrix(const sensor_datastructures::OdomData& odom_data) const;

        /**
         * @brief Get the observation matrix object
         * 
         * @param imu_data          The IMU data
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_observation_matrix(const sensor_datastructures::IMUData& imu_data) const;

        /**
         * @brief Get the measurement covariance matrix object
         * 
         * @param odom_data          The Odom Data
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_measurement_covariance_matrix(const sensor_datastructures::OdomData& odom_data) const;

        /**
         * @brief Get the measurement covariance matrix object
         * 
         * @param imu_data            The IMU data
         * @return Eigen::MatrixXd 
         */
        Eigen::MatrixXd get_measurement_covariance_matrix(const sensor_datastructures::IMUData& imu_data) const;



        private:
            const double noise_ax;    // acceleration process noise in x
            const double noise_ay;    // acceleration process noise in y
            const double noise_atheta;// acceleration process noise in theta
    };
}

#endif