#ifndef KALMAN_FILTER__KALMAN_FILTER_HPP
#define KALMAN_FILTER__KALMAN_FILTER_HPP

#include<eigen3/Eigen/Dense>

namespace kalman_filter{

/** @brief Kalman Filter is a linear filter that localizes the object
 *         given a sequence of sensor data inputs
 * 
*/
class KalmanFilter{
    
    public:

        /**
         * @brief Constructor for the KalmanFilter object
        */
        KalmanFilter();

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
         * @param[in]   noise_ax    Process x noise
         * @param[in]   noise_ay    Process y noise
         * 
         * @return void
        */
        void update_timestamp_changes(const double dt, const int noise_ax, const int noise_ay);

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
         * 
         * @return  void
        */
        void update(const Eigen::VectorXd &z);

    private:
        // State Vector
        Eigen::VectorXd x_;

        // State Transition Matrix
        Eigen::MatrixXd F_;

        // State Covariance Matrix
        Eigen::MatrixXd P_;

        // Process Covariance Matrix
        Eigen::MatrixXd Q_;

        // Measurement Matrix
        Eigen::MatrixXd H_;

        // Measurement Covariance Matrix
        Eigen::MatrixXd R_;

};

}

#endif // end of KALMAN_FILTER_H