#ifndef KALMAN_FILTER_TRACKER_HPP
#define KALMAN_FILTER_TRACKER_HPP

#include <measurement_package.hpp>
#include <kalman_filter.hpp>

namespace kalman_filter {
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
      void measurement_update(const kalman_filter::MeasurementPackage &measurement_pack);

      bool is_initialized;
      int64_t previous_timestamp;
      kalman_filter::KalmanFilter kf;
      int noise_ax;
      int noise_ay;
  };
}

#endif