
#include <eigen3/Eigen/Dense>
#include <track_filter.hpp>
#include <converters/converters.hpp>
#include <core_datastructures/euler_axis.hpp>
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

    void Tracker::measurement_update_IMU(const sensor_datastructures::IMUData& imu_data, float timestamp){
        if(!is_initialized){
            // initialization
            Eigen::VectorXd initial_state(9, 1);
            initial_state.setZero();
            kf.init_state(initial_state);
            previous_timestamp = timestamp;
            is_initialized = true;
            return;
        }
        float dt = (timestamp-previous_timestamp)/1000000.0;
        previous_timestamp = timestamp;

        kf.update_timestamp_changes(dt);
        kf.predict();
        
        Eigen::MatrixXd H = kf.dynamic_model.get_observation_matrix(imu_data);
        Eigen::MatrixXd R = kf.dynamic_model.get_measurement_covariance_matrix(imu_data);

        core_datastructures::EulerAngle euler_angle = common_domain::converters::quat_to_eul(imu_data.orientation);

        Eigen::VectorXd z(4);
        z << euler_angle.yaw, imu_data.angular_velocity(2), imu_data.linear_acceleration(0), imu_data.linear_acceleration(1);

        kf.update(z, H, R);
        states = kf.get_states();
        // std::cout << states << std::endl;
    }

    void Tracker::measurement_update_Odom(const sensor_datastructures::OdomData& odom_data, float timestamp){
        if(!is_initialized){
            // initialization
            Eigen::VectorXd initial_state(9, 1);
            initial_state.setZero();
            kf.init_state(initial_state);
            previous_timestamp = timestamp;
            is_initialized = true;
            return;
        }
        float dt = (timestamp-previous_timestamp)/1000000.0;
        previous_timestamp = timestamp;

        kf.update_timestamp_changes(dt);
        kf.predict();

        Eigen::MatrixXd H = kf.dynamic_model.get_observation_matrix(odom_data);
        Eigen::MatrixXd R = kf.dynamic_model.get_measurement_covariance_matrix(odom_data);

        core_datastructures::EulerAngle euler_angle = common_domain::converters::quat_to_eul(odom_data.orientation);

        Eigen::VectorXd z(6);
        z << odom_data.position(0), odom_data.position(1), euler_angle.yaw, odom_data.linear_velocity(0), odom_data.linear_velocity(1), odom_data.angular_velocity(2);

        kf.update(z, H, R);
        states = kf.get_states();
    }

    // void Tracker::measurement_update(const MeasurementPackage& measurement_pack){
       
    //     if(!is_initialized){

    //         Eigen::VectorXd initial_state(4);
    //         if (measurement_pack.sensor_type == SensorType::LASER) {
    //             initial_state << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    //         }
    //         else {
    //             double rho = measurement_pack.raw_measurements_[0];
    //             double phi = measurement_pack.raw_measurements_[1];
    //             double rho_dot = measurement_pack.raw_measurements_[2];

    //             double px = rho * cos(phi);
    //             double py = rho * sin(phi);
    //             double vx = rho_dot * cos(phi);
    //             double vy = rho_dot * sin(phi);
    //             initial_state << px, py, vx, vy;
    //         }
    //         previous_timestamp = measurement_pack.timestamp_;
    //         kf.init_state(initial_state);
    //         is_initialized = true;
    //         return;
    //     }
    //     float dt = (measurement_pack.timestamp_-previous_timestamp)/1000000.0;
    //     previous_timestamp = measurement_pack.timestamp_;

    //     kf.update_timestamp_changes(dt,noise_ax,noise_ay);
    //     kf.predict();
    //     Eigen::MatrixXd H = kf.get_H(measurement_pack.sensor_type);
    //     Eigen::MatrixXd R = kf.get_R(measurement_pack.sensor_type);
    //     kf.update(measurement_pack.raw_measurements_, H, R);
    //     states = kf.get_states();
    // }
}
