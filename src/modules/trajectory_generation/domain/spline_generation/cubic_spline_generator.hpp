#ifndef CUBIC_SPLINE_GENERATOR_HPP
#define CUBIC_SPLINE_GENERATOR_HPP

#include <i_spline_generator.hpp>
#include <eigen3/Eigen/Dense>

namespace trajectory_generation {
    class CubicSplineGenerator {
        public:
            CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal);
            /*
                Generate splines as per given end posture
                @param[in] core_datastructures::Posture posture
            */
            std::vector<core_datastructures::Posture> get_spline();

            std::vector<double> calculate_parameters(double p1, double p2, double s);

            core_datastructures::Posture get_next_state(std::vector<double>& params, double arc_length);

            Eigen::Matrix3d calculate_Jacobian(std::vector<double>& params);


            void run_gradient_descent();

            std::vector<core_datastructures::Posture> generate_splines();

            double calculate_kappa(std::vector<double>& params, double s);
            double calculate_theta(std::vector<double>& params, double s);

            // double calculate_x();
            double calculate_x(std::vector<double>& params, double s);

            double calculate_y(std::vector<double>& params, double s);
                
        private:
            double P0;
            double P1;
            double P2;
            double P3;
            double SG;
            Eigen::Vector3d q_thresh;
            core_datastructures::Posture start;
            core_datastructures::Posture goal;
    };
}


#endif  /*CUBIC_SPLINE_GENERATOR_HPP*/