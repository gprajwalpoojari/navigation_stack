
#include<eigen3/Eigen/Dense>
#include <track_filter.hpp>

namespace kalman_filter {

    Tracker::Tracker(){
        is_initialized = false;
        previous_timestamp = 0;
        noise_ax = 5;
        noise_ay = 5;

        // Uncomment the following line during testing

        // kf.print_matrices();
    }

    void Tracker::measurement_update(const MeasurementPackage& measurement_pack){
        if(!is_initialized){

            Eigen::VectorXd initial_state(4);
            initial_state << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            previous_timestamp = measurement_pack.timestamp_;
            kf.init_state(initial_state);
            is_initialized = true;
            return;
        }

        float dt = (measurement_pack.timestamp_-previous_timestamp)/1000000.0;
        previous_timestamp = measurement_pack.timestamp_;

        kf.update_timestamp_changes(dt,noise_ax,noise_ay);
        kf.predict();
        kf.update(measurement_pack.raw_measurements_);
    }
}
