#ifndef KALMAN_FILTER_TRACKER_H
#define KALMAN_FILTER_TRACKER_H

#include "Measurement_Package.hpp"
#include "Kalman_Filter.hpp"

class Tracker{
  public:
   
    bool is_initialized;
    int64_t previous_timestamp;
    Kalman_Filter::KalmanFilter kf;
    int noise_ax;
    int noise_ay;
    
    Tracker();

    // virtual ~Tracker();

    void measurement_update(const MeasurementPackage &measurement_pack);




};



#endif