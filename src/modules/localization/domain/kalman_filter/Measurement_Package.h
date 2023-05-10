#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include<eigen3/Eigen/Dense>

class MeasurementPackage{
    public:

    enum Sensor_Type{
        LASER,RADAR
    } sensor_type;

    Eigen::VectorXd raw_measurements_;
    int64_t timestamp_; 

};


#endif // End of MEASUREMENT_PACKAGE_H