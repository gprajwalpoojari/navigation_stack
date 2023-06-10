#ifndef SENSOR_DATASTRUCTURES__IMU_HPP
#define SENSOR_DATASTRUCTURES__IMU_HPP

#include <eigen3/Eigen/Geometry>

namespace sensor_datastructures{
    struct IMUData{
        Eigen::Quaterniond orientation;                 // orientation
        Eigen::Matrix3d orientation_covariance;         // orientation covariance

        Eigen::Vector3d angular_velocity;               // angular velocity
        Eigen::Matrix3d angular_velocity_covariance;    // angular velocity covariance

        Eigen::Vector3d linear_acceleration;            // linear acceleration
        Eigen::Matrix3d linear_acceleration_covariance; // linear acceleration covariance

    };

}

#endif