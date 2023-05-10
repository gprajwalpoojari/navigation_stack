#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include<eigen3/Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace Kalman_Filter{

class KalmanFilter{

     // State Vector
    VectorXd x_;

    // State Transition Matrix
    MatrixXd F_;

    // State Covariance Matrix
    MatrixXd P_;

    // Process Covariance Matrix
    MatrixXd Q_;

    // Measurement Matrix
    MatrixXd H_;

    // Measurement Covariance Matrix
    MatrixXd R_;
    
    public:
    
    KalmanFilter();

    void init_state(VectorXd start); 

    void update_timestamp_changes(const float dt, const int noise_ax, const int noise_ay);

    void print_matrices();

    void predict();

    void update(const VectorXd &z);

    // virtual ~KalmanFilter();

};

}

#endif // end of KALMAN_FILTER_H