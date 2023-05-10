#ifndef KALMAN_FILTER_DYNAMICS_H
#define KALMAN_FILTER_DYNAMICS_H

#include<eigen3/Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class Dynamics{
    public:
    
    // Dynamics();

    void set_matrix_values(VectorXd &x, MatrixXd &F, MatrixXd &P, MatrixXd &Q, MatrixXd &H, MatrixXd &R);

    // virtual ~Dynamics();



};


#endif