#include <iostream>
#include <kalman_filter.hpp>
#include <dynamics.hpp>

namespace kalman_filter {

    KalmanFilter::KalmanFilter(){
        Dynamics model;
        model.set_matrix_values(x_,F_,P_,Q_,H_,R_);
    }

    void KalmanFilter::init_state(Eigen::VectorXd start){
        x_ = start;
    }

    void KalmanFilter::update_timestamp_changes(const double dt, const int noise_ax, const int noise_ay){

        F_(0,2) = dt;
        F_(1,3) = dt;

        double dt_2 = dt*dt;
        double dt_3 = dt_2*dt;
        double dt_4 = dt_3*dt;

        Q_<< dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
                0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ax / 2,
                dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
                0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;


    }


    // Uncomment the following lines during testing

    // void KalmanFilter::print_matrices(){
    //     std::cout<<"x_ is:" << std::endl;
    //     std::cout<< R_ <<std::endl;
    // }

    void KalmanFilter::predict(){
        x_ = F_*x_;
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_*P_*Ft + Q_;

    }

    void KalmanFilter::update(const Eigen::VectorXd &z){
        Eigen::VectorXd z_pred = H_*x_;
        Eigen::VectorXd err = z - z_pred;
        Eigen::MatrixXd Ht = H_.transpose();
        Eigen::MatrixXd S = H_*P_*Ht + R_;
        Eigen::MatrixXd Si = S.inverse();
        Eigen::MatrixXd Kn = P_*Ht*Si;

        //New Estimate Updation Equation
        x_ = x_ + Kn*(err);
        long x_size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size,x_size);
        P_ = (I-Kn*H_)*P_*((I-Kn*H_).transpose())+(Kn*R_*Kn.transpose());


        // Uncomment the following lines during testing

        // std::cout << "New_state:" << std::endl;
        // std::cout << x_ << std::endl;
        // std::cout << "New Uncertainity:" << std::endl;
        // std::cout << P_ << std::endl;

    }
}