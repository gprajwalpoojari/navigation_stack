#include <dynamics.hpp>

namespace sensor_fusion::extended_kalman_filter {
     void Dynamics::set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, Eigen::MatrixXd &Q){
     // Set Matrix Values

          x = Eigen::VectorXd(4);

          // state covariance matrix P
          P = Eigen::MatrixXd(4, 4);
          P << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

          // the initial transition matrix F_
          F = Eigen::MatrixXd(4, 4);
          F << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

          Q = Eigen::MatrixXd(4,4);
     }
}
