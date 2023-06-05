#ifndef COMMON__CORE_DATASTRUCTURES__RADAR_HPP
#define COMMON__CORE_DATASTRUCTURES__RADAR_HPP

#include <eigen3/Eigen/Dense>

namespace common::core_datastructures {
    struct Radar {
        double rho;   // raw measurement rho
        double phi;   // raw measurement phi
        double rho_dot; // raw measurement rho_dot

        Eigen::Matrix2d R;  // The measurement covariance
        R << 0.0225, 0, 0,
             0, 0.0225, 0,
             0, 0, 0.0225;
    };
}

#endif  /*COMMON__CORE_DATASTRUCTURES__RADAR_HPP*/