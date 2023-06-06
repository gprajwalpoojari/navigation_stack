#ifndef SENSOR_FUSION__EXTENDED_KALMAN_FILTER_TRACKER_HPP
#define SENSOR_FUSION__EXTENDED_KALMAN_FILTER_TRACKER_HPP

#include <extended_kalman_filter.hpp>

namespace sensor_fusion::extended_kalman_filter {
  class Tracker{
    public:
    
      /**
       * @brief Constructor for Tracker object
      */
      Tracker();

      /**
       * @brief Update measurement
       * 
       * @param[in] measurement_pack The measurement package
       * 
       * @return void
      */
      void measurement_update(const MeasurementPackage &measurement_pack);

      bool is_initialized;
      int64_t previous_timestamp;
      ExtendedKalmanFilter kf;
      int noise_ax;
      int noise_ay;
  };
}

#endif