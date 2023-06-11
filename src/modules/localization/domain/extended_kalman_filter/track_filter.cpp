
#include <eigen3/Eigen/Dense>
#include <track_filter.hpp>
#include <iostream>

namespace localization::extended_kalman_filter {

    Tracker::Tracker(){
        is_initialized = false;
        previous_timestamp = 0;
        noise_ax = 5;
        noise_ay = 5;

        // Uncomment the following line during testing

        // kf.print_matrices();
    }

    void Tracker::measurement_update_IMU(const sensor_datastructures::IMUData& imu_data){
        if(!is_initialized){
            // initialization
            return;
        }
        float dt;

        kf.update_timestamp_changes(dt,noise_ax,noise_ay);
        kf.predict();
        
        Eigen::MatrixXd H;
        Eigen::MatrixXd R;
        Eigen::MatrixXd z;

        kf.update(z, H, R);
        states = kf.get_states();
    }

    void Tracker::measurement_update_Odom(const sensor_datastructures::OdomData& odom_data){
        if(!is_initialized){
            // initialization
            return;
        }
        float dt;

        kf.update_timestamp_changes(dt,noise_ax,noise_ay);
        kf.predict();

        Eigen::MatrixXd H;
        Eigen::MatrixXd R;
        Eigen::MatrixXd z;

        kf.update(z, H, R);
        states = kf.get_states();
    }

    void Tracker::measurement_update(const MeasurementPackage& measurement_pack){
       
        if(!is_initialized){

            Eigen::VectorXd initial_state(4);
            if (measurement_pack.sensor_type == SensorType::LASER) {
                initial_state << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            }
            else {
                double rho = measurement_pack.raw_measurements_[0];
                double phi = measurement_pack.raw_measurements_[1];
                double rho_dot = measurement_pack.raw_measurements_[2];

                double px = rho * cos(phi);
                double py = rho * sin(phi);
                double vx = rho_dot * cos(phi);
                double vy = rho_dot * sin(phi);
                initial_state << px, py, vx, vy;
            }
            previous_timestamp = measurement_pack.timestamp_;
            kf.init_state(initial_state);
            is_initialized = true;
            return;
        }
        float dt = (measurement_pack.timestamp_-previous_timestamp)/1000000.0;
        previous_timestamp = measurement_pack.timestamp_;

        kf.update_timestamp_changes(dt,noise_ax,noise_ay);
        kf.predict();
        Eigen::MatrixXd H = kf.get_H(measurement_pack.sensor_type);
        Eigen::MatrixXd R = kf.get_R(measurement_pack.sensor_type);
        kf.update(measurement_pack.raw_measurements_, H, R);
        states = kf.get_states();
    }
}
