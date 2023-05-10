#ifndef CUBIC_SPLINE_GENERATOR_HPP
#define CUBIC_SPLINE_GENERATOR_HPP

#include <spline_generation/i_spline_generator.hpp>
#include <eigen3/Eigen/Dense>

namespace trajectory_generation {
    class CubicSplineGenerator {
        public:
            CubicSplineGenerator();
            CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal);
            /*
                Generate splines as per given end posture
                @param[in] core_datastructures::Posture posture
            */
            std::vector<core_datastructures::Posture> get_spline();

            std::vector<double> calculate_parameters(double P1, double P2, double P3);

            core_datastructures::Posture get_next_state(std::vector<double>& params, double SG);

            std::vector<std::vector<double>> calculate_Jacobian(std::vector<double>& params, double SG);


            void run_gradient_descent();

            double get_delta(const core_datastructures::Posture& node1, const core_datastructures::Posture& node2);

            std::vector<core_datastructures::Posture> generate_spline();

            double calculate_kappa(std::vector<double>& params, double SG);
            double calculate_theta(std::vector<double>& params, double SG);

            // double calculate_x();
            double calculate_x(std::vector<double>& params, double SG);

            double calculate_y(std::vector<double>& params, double SG);

        
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