#ifndef COMMON__CORE_DATASTRUCTURES__LASER_HPP
#define COMMON__CORE_DATASTRUCTURES__LASER_HPP

#include <eigen3/Eigen/Dense>

namespace common::core_datastructures {
    struct Laser {
        double x;   // raw measurement x
        double y;   // raw measurement y

        Eigen::Matrix2d R;  // The measurement covariance
        R << 0.0225, 0,
             0, 0.0225;

    };
}

#endif  /*COMMON__CORE_DATASTRUCTURES__LASER_HPP*/