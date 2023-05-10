#include <iostream>
#include "Kalman_Filter.hpp"
#include "Dynamics.hpp"

using Kalman_Filter::KalmanFilter;

KalmanFilter::KalmanFilter(){
    Dynamics model;
    model.set_matrix_values(x_,F_,P_,Q_,H_,R_);

}

void KalmanFilter::init_state(VectorXd start){
    x_ = start;
}

void KalmanFilter::update_timestamp_changes(const float dt, const int noise_ax, const int noise_ay){

    F_(0,2) = dt;
    F_(1,3) = dt;

    float dt_2 = dt*dt;
    float dt_3 = dt*dt*dt;
    float dt_4 = dt*dt*dt*dt;

    Q_<< dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
            0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ax / 2,
            dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
            0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;


}

void KalmanFilter::print_matrices(){
    std::cout<<"x_ is:"<<R_<<std::endl;
}

void KalmanFilter::predict(){
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::update(const VectorXd &z){
    VectorXd z_pred = H_*x_;
    VectorXd err = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_*P_*Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Kn = P_*Ht*Si;

    //New Estimate Updation Equation
    x_ = x_ + Kn*(err);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size,x_size);
    P_ = (I-Kn*H_)*P_*((I-Kn*H_).transpose())+(Kn*R_*Kn.transpose());

    std::cout<<"New_state:"<<x_<<std::endl;
    std::cout<<"New Uncertainity"<<P_<<std::endl;

}

// KalmanFilter::~KalmanFilter(){

// }