#include <dynamics.hpp>

namespace kalman_filter {
     void Dynamics::set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, Eigen::MatrixXd &Q, Eigen::MatrixXd &H, Eigen::MatrixXd &R){
     // Set Matrix Values

          x = Eigen::VectorXd(4);

          // state covariance matrix P
          P = Eigen::MatrixXd(4, 4);
          P << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;


          // measurement covariance
          R = Eigen::MatrixXd(2, 2);
          R << 0.0225, 0,
                         0, 0.0225;

          // measurement matrix
          H = Eigen::MatrixXd(2, 4);
          H << 1, 0, 0, 0,
               0, 1, 0, 0;

          // the initial transition matrix F_
          F = Eigen::MatrixXd(4, 4);
          F << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

          Q = Eigen::MatrixXd(4,4);
     }
}
