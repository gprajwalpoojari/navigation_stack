#include "Dynamics.hpp"

// Dynamics(){}

void Dynamics::set_matrix_values(VectorXd &x, MatrixXd &F, MatrixXd &P, MatrixXd &Q, MatrixXd &H, MatrixXd &R){
    // Set Matrix Values

        x = VectorXd(4);

        // state covariance matrix P
        P = MatrixXd(4, 4);
        P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;


        // measurement covariance
        R = MatrixXd(2, 2);
        R << 0.0225, 0,
                    0, 0.0225;

        // measurement matrix
        H = MatrixXd(2, 4);
        H << 1, 0, 0, 0,
             0, 1, 0, 0;

        // the initial transition matrix F_
        F = MatrixXd(4, 4);
        F << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

        Q = MatrixXd(4,4);
}

// Dynamics::~Dynamics(){

// }