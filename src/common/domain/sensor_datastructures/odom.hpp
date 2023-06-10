#ifndef SENSOR_DATASTRUCTURES__ODOM_HPP
#define SENSOR_DATASTRUCTURES__ODOM_HPP

#include <eigen3/Eigen/Geometry>

namespace sensor_datastructures {
    struct OdomData {
        // Pose information with covariance
        Eigen::Matrix3d position;               // position
        Eigen::Quaterniond orientation;         // orientation

        Eigen::MatrixXd pose_covariance;        // pose covariance

        // Twist information with covariance
        Eigen::Matrix3d linear_velocity;        // linear velocity
        Eigen::Matrix3d angular_velocity;       // angular velocity

        Eigen::MatrixXd twist_covariance;       // twist covariance
    };
}

#endif