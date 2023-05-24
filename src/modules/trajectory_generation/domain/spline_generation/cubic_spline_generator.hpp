#ifndef CUBIC_SPLINE_GENERATOR_HPP
#define CUBIC_SPLINE_GENERATOR_HPP

#include <i_spline_generator.hpp>
#include <eigen3/Eigen/Dense>

namespace trajectory_generation {
    /**
     * @brief CubicSplineGenerator Class for generating cubic splines given two postures
     * 
     */
    class CubicSplineGenerator {
        public:
            /**
             * @brief Construct a new Cubic Spline Generator object
             * 
             * @param start core_datastructures::Posture spline start posture
             * @param goal core_datastructures::Posture spline end posture
             */
            CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal);
            
            /**
             * @brief Get the spline object
             * 
             * @return std::vector<core_datastructures::Posture> containing spline points
             */
            std::vector<core_datastructures::Posture> get_spline();

            /**
             * @brief Computes the spline parameters given [P1, P2, SG] values
             * 
             * @param p1 
             * @param p2 
             * @param s 
             * @return std::vector<double> the spline parameters {a, b, c, d}
             */
            std::vector<double> calculate_parameters(double p1, double p2, double s);

            /**
             * @brief Get the next state object
             * 
             * @param params 
             * @param s 
             * @return core_datastructures::Posture return next state given spline params
             */
            core_datastructures::Posture get_next_state(const std::vector<double>& params, double s);


            /**
             * @brief Computes Jacobian matrix for the given params
             * 
             * @param params 
             * @return Eigen::Matrix3d Jacobian Matrix
             */
            Eigen::Matrix3d calculate_Jacobian(std::vector<double>& params);


            /**
             * @brief Runs graident descent to compute final spline parameters
             * 
             */
            void run_gradient_descent();

            /**
             * @brief Outputs a set of postures after computing the final spline params
             * 
             * @return std::vector<core_datastructures::Posture> 
             */
            std::vector<core_datastructures::Posture> generate_splines();

            /**
             * @brief Computes curvature given params and arc length
             * 
             * @param params 
             * @param s 
             * @return double Curvature
             */
            double calculate_kappa(const std::vector<double>& params, double s);

            /**
             * @brief Computes theta given params and arc length
             * 
             * @param params 
             * @param s 
             * @return double theta value in radians
             */
            double calculate_theta(const std::vector<double>& params, double s);

            // double calculate_x();
            /**
             * @brief computes x given params and arc length
             * 
             * @param params 
             * @param s 
             * @return double 
             */
            double calculate_x(const std::vector<double>& params, double s);

            /**
             * @brief Computes y given params and arc length
             * 
             * @param params 
             * @param s 
             * @return double 
             */
            double calculate_y(const std::vector<double>& params, double s);

            /**
             * @brief Gets the euclidian distance given two postures
             * 
             * @param start 
             * @param goal 
             * @return double eucilidian distance
             */
            double get_distance(const core_datastructures::Posture& start, const core_datastructures::Posture& goal);

            
            /**
             * @brief return new vector | goal - start |
             * 
             * @param start 
             * @param goal 
             * @return Eigen::Vector3d 
             */
            Eigen::Vector3d diff(const core_datastructures::Posture& start, const core_datastructures::Posture& goal);
            
            /**
             * @brief checks if qiven vector is greater than threhold
             * 
             * @param v1  
             * @return true if values within range of threhold
             * @return false if values greater than the threshold
             */
            bool comp(Eigen::Vector3d& v1);

        private:
            double P0;
            double P1;
            double P2;
            double P3;
            double SG;
            
            /**
             * @brief Spline convergence threshold value
             * 
             */
            Eigen::Vector3d q_thresh;

            /**
             * @brief Relative Goal posture for spline generation
             * 
             */
            core_datastructures::Posture goal;
    };
}


#endif  /*CUBIC_SPLINE_GENERATOR_HPP*/