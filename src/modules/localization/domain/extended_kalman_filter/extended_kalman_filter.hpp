#ifndef LOCALIZATION__EXTENDED_KALMAN_FILTER__EXTENDED_KALMAN_FILTER_HPP
#define LOCALIZATION__EXTENDED_KALMAN_FILTER__EXTENDED_KALMAN_FILTER_HPP

#include<eigen3/Eigen/Dense>
#include <measurement_package.hpp>
#include <constant_acceleration_model.hpp>


namespace localization::extended_kalman_filter{

/** @brief Kalman Filter is a linear filter that localizes the object
 *         given a sequence of sensor data inputs
 * 
*/
class ExtendedKalmanFilter{
    
    public:


        // Object of Dynamic Model
        localization::dynamic_model::ConstantAccelerationModel dynamic_model;

        /**
         * @brief Constructor for the KalmanFilter object
        */
        ExtendedKalmanFilter();

        /**
         * @brief Initialize the current state
         * 
         * @param[in] start Initial state
         * 
         * @return void
        */
        void init_state(Eigen::VectorXd start); 

        /**
         * @brief Update the process noise matrix depending on the time stamp
         * 
         * @param[in]   dt    The time stamp
         * 
         * @return void
        */
        void update_timestamp_changes(const double dt);

        // Uncomment the following lines during testing

        // /**
        //  * @brief Utility function for printing matrices
        // */
        // void print_matrices();

        /**
         * @brief   The prediction logic of kalman filter
         * 
         * @return void
        */
        void predict();

        /**
         * @brief   The update logic of kalman filter
         * 
         * @param[in]   z   The sensor measurement
         * @param[in]   H   The observation matrix for the sensor
         * @param[in]   R   The covariance matrix for the sensor
         * 
         * @return  void
        */
        void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

        /**
         * @brief   Computes the jacobian for radar
         * 
         * @param[in]   x   The predicted state
         * 
         * @return  Eigen::MatrixXd
        */
        Eigen::MatrixXd compute_jacobian(const Eigen::VectorXd& x) const;

        /**
         * @brief Gets the measurement matrix given a sensor type
         * 
         * @param[in]   sensor_type The type of sensor
         * 
         * @return      Eigen::MatrixXd     The measurement matrix
        */
        Eigen::MatrixXd get_H(SensorType sensor_type) const;

        /**
         * @brief Gets the measurement matrix given a sensor type
         * 
         * @param[in]   sensor_type The type of sensor
         * 
         * @return      Eigen::MatrixXd     The measurement matrix
        */
        Eigen::MatrixXd get_R(SensorType sensor_type) const;

        /**
         * @brief Get the states of object
         * 
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd get_states() const;

    private:
        // State Vector
        Eigen::VectorXd x_;

        // Control input
        Eigen::VectorXd u_;

        // State Transition Matrix
        Eigen::MatrixXd F_;

        // Control input Matrix
        Eigen::MatrixXd G_;

        // State Covariance Matrix
        Eigen::MatrixXd P_;

        // Process Covariance Matrix
        Eigen::MatrixXd Q_;


};

}

#endif // LOCALIZATION__EXTENDED_KALMAN_FILTER__EXTENDED_KALMAN_FILTER_HPP