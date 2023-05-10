#ifndef TRACKER_H
#define TRACKER_H

#include "Measurement_Package.h"
#include "Kalman_Filter.h"

class Tracker{
  public:
   
    bool is_initialized;
    int64_t previous_timestamp;
    KalmanFilter kf;
    int noise_ax;
    int noise_ay;
    
    Tracker();

    virtual ~Tracker();

    void measurement_update(const MeasurementPackage &measurement_pack);




};



#endif