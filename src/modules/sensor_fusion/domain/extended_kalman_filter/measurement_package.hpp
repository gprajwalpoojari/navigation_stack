#ifndef SENSOR_FUSION__EXTENDED_KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP
#define SENSOR_FUSION__EXTENDED_KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP

#include<eigen3/Eigen/Dense>
#include <sensor_type.hpp>

namespace sensor_fusion::extended_kalman_filter {

    class MeasurementPackage{
        
        public:
            SensorType sensor_type;

            Eigen::VectorXd raw_measurements_;
            int64_t timestamp_; 
    };
}


#endif // End of SENSOR_FUSION__EXTENDED_KALMAN_FILTER__MEASUREMENT_PACKAGE_HPP