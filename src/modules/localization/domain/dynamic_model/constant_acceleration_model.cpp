#include <constant_acceleration_model.hpp>
#include <converters/converters.hpp>
namespace localization::dynamic_model {

    ConstantAccelerationModel::ConstantAccelerationModel(double noise_ax, double noise_ay, double noise_atheta) :
                                                         noise_ax(noise_ax), noise_ay(noise_ay), 
                                                         noise_atheta(noise_atheta) {}

    void ConstantAccelerationModel::set_matrix_values(Eigen::VectorXd &x, Eigen::MatrixXd &F, Eigen::MatrixXd &P, Eigen::MatrixXd &Q){
        // Set Matrix Values

        x = Eigen::VectorXd(9);
          // state covariance matrix P
        P = Eigen::MatrixXd(9, 9);
        P <<  100, 0, 0, 0, 0, 0, 0, 0, 0, 
              0, 100, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 100, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 100, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 100, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 100, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 100, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 100, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 100;

        // the initial transition matrix F_
        double dt = 0;
        F = Eigen::MatrixXd(9, 9);
        update_state_matrix(F, dt);
        Q = Eigen::MatrixXd(9, 9);
        update_process_noise_matrix(Q, dt);
    }

    void ConstantAccelerationModel::update_process_noise_matrix(Eigen::MatrixXd& Q, double dt) {
        double dt_2 = dt*dt;
        double dt_3 = dt_2*dt;
        double dt_4 = dt_3*dt;

        Q<<  dt_4*noise_ax/4, 0,               0,                   dt_3*noise_ax/2, 0,               0,                   dt_2*noise_ax/2, 0,               0,
                0,               dt_4*noise_ay/4, 0,                   0,               dt_3*noise_ay/2, 0,                   0,               dt_2*noise_ay/2, 0,
                0,               0,               dt_4*noise_atheta/4, 0,               0,               dt_3*noise_atheta/2, 0,               0,               dt_2*noise_atheta/2,
                dt_3*noise_ax/2, 0,               0,                   dt_2 * noise_ax, 0,               0,                   dt*noise_ax,     0,               0,
                0,               dt_3*noise_ay/2, 0,                   0,               dt_2*noise_ay,   0,                   0,               dt*noise_ay,     0,
                0,               0,               dt_3*noise_atheta/2, 0,               0,               dt_2*noise_atheta,   0,               0,               dt*noise_atheta,
                dt_2*noise_ax/2, 0,               0,                   dt*noise_ax,     0,               0,                   noise_ax,        0,               0,
                0,               dt_2*noise_ay/2, 0,                   0,               dt*noise_ay,     0,                   0,               noise_ay,        0,
                0,               0,               dt_2*noise_atheta/2, 0,               0,               dt*noise_atheta,     0,               0,               noise_atheta;

    }

    void ConstantAccelerationModel::update_state_matrix(Eigen::MatrixXd& F, double dt) {

        double temp = 0.5 * std::pow(dt, 2);
        F << 1, 0, 0, dt, 0, 0, temp, 0, 0,     // x
             0, 1, 0, 0, dt, 0, 0, temp, 0,     // y 
             0, 0, 1, 0, 0, dt, 0, 0, temp,     // theta (yaw)
             0, 0, 0, 1, 0, 0, dt, 0, 0,        // x_dot
             0, 0, 0, 0, 1, 0, 0, dt, 0,        // y_dot
             0, 0, 0, 0, 0, 1, 0, 0, dt,        // theta_dot
             0, 0, 0, 0, 0, 0, 1, 0, 0,         // x_dot_dot
             0, 0, 0, 0, 0, 0, 0, 1, 0,         // y_dot_dot
             0, 0, 0, 0, 0, 0, 0, 0, 1;         // theta_dot_dot
        
    }

    Eigen::MatrixXd ConstantAccelerationModel::get_observation_matrix(const sensor_datastructures::OdomData& odom_data) const{
        //z_predicted = [x y z yaw x_dot, y_dot, yaw_dot] 6x1

        // x = [x y theta x_dot y_dot_theta_dot x_dot_dot y_dot_dot theta_dot_dot]  9x1
        // z_predicted = H * x;
        Eigen::Matrix<double, 6, 9> H = Eigen::MatrixXd::Zero(6, 9);
        H.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6);
        return H;
    }

    Eigen::MatrixXd ConstantAccelerationModel::get_measurement_covariance_matrix(const sensor_datastructures::OdomData& odom_data) const {
        Eigen::Matrix<double, 6, 6> R;
        R << odom_data.pose_covariance(0,0), odom_data.pose_covariance(0, 1), odom_data.pose_covariance(0,5), 0, 0, 0,
             odom_data.pose_covariance(1,0), odom_data.pose_covariance(1, 1), odom_data.pose_covariance(1,5), 0, 0, 0,
             odom_data.pose_covariance(5,0), odom_data.pose_covariance(5, 1), odom_data.pose_covariance(5,5), 0, 0, 0,
             0, 0, 0, odom_data.twist_covariance(0,0), odom_data.twist_covariance(0, 1), odom_data.twist_covariance(0,5),
             0, 0, 0, odom_data.twist_covariance(1,0), odom_data.twist_covariance(1, 1), odom_data.twist_covariance(1,5),
             0, 0, 0, odom_data.twist_covariance(5,0), odom_data.twist_covariance(5, 1), odom_data.twist_covariance(5,5);
        return R;
    }

    Eigen::MatrixXd ConstantAccelerationModel::get_observation_matrix(const sensor_datastructures::IMUData& imu_data) const{
        Eigen::Matrix<double, 4, 9> H;
        H << 0, 0, 1, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0, 0;

        return H;
    }

    Eigen::MatrixXd ConstantAccelerationModel::get_measurement_covariance_matrix(const sensor_datastructures::IMUData& imu_data) const{
        Eigen::Matrix<double, 4, 4> R;
        
        R << imu_data.orientation_covariance(2, 2), 0, 0, 0,
             0, imu_data.angular_velocity_covariance(2, 2), 0, 0,
             0, 0, imu_data.linear_acceleration_covariance(0, 0), imu_data.linear_acceleration_covariance(0, 1),
             0, 0, imu_data.linear_acceleration_covariance(1, 0), imu_data.linear_acceleration_covariance(1, 1);
             
        return R;
    }




}
