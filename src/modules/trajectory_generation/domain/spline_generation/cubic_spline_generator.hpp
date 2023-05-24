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
            CubicSplineGenerator(const core_datastructures::Posture& start, const core_datastructures::Posture& goal);
            
            /**
             * @brief Get the spline object
             * 
             * @return std::vector<core_datastructures::Posture> containing spline points
             */
            std::vector<core_datastructures::Posture> get_spline();

            /**
             * @brief Computes the spline coefficients given [P1, P2, SG] storted in perturb_params variable
             * 
             * @return const std::vector<double> the spline coefficients {a, b, c, d}
             */
            const std::vector<double> calculate_spline_coefficients() const;

            /**
             * @brief Get the next state object
             * 
             * @param[in] coeffs    The spline coefficients {a, b, c, d}
             * @param[in] s         The arc length
             * @return const core_datastructures::Posture return next state given spline params
             */
            const core_datastructures::Posture get_next_state(const std::vector<double>& coeffs, double s) const;


            /**
             * @brief Computes Jacobian matrix for the given params
             * 
             * @param[in] coeffs    The spline coefficients {a, b, c, d}
             * @return              Eigen::Matrix3d Jacobian Matrix
             */
            Eigen::Matrix3d calculate_Jacobian(const std::vector<double>& coeffs);


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
            const std::vector<core_datastructures::Posture> generate_splines() const;

            /**
             * @brief Computes curvature given params and arc length
             * 
             * @param  coeffs   The spline coefficients {a, b, c, d}
             * @param s         The arc length
             * @return double Curvature
             */
            double calculate_kappa(const std::vector<double>& coeffs, double s) const;

            /**
             * @brief Computes theta given coeffs and arc length
             * 
             * @param coeffs    The spline coefficients {a, b, c, d}
             * @param s         The arc length
             * @return double theta value in radians
             */
            double calculate_theta(const std::vector<double>& coeffs, double s) const;

            // double calculate_x();
            /**
             * @brief computes x given coeffs and arc length
             * 
             * @param coeffs    The spline coefficients {a, b, c, d}
             * @param s         The arc length
             * @return double 
             */
            double calculate_x(const std::vector<double>& coeffs, double s) const;

            /**
             * @brief Computes y given coeffs and arc length
             * 
             * @param coeffs    The spline coefficients {a, b, c, d}
             * @param s         The arc length
             * @return double 
             */
            double calculate_y(const std::vector<double>& coeffs, double s) const;

            /**
             * @brief Gets the euclidian distance given two postures
             * 
             * @param start     The start state
             * @param goal      The goal state
             * @return          double eucilidian distance
             */
            double get_distance(const core_datastructures::Posture& start, const core_datastructures::Posture& goal) const;

            
            /**
             * @brief return new vector | goal - start |
             * 
             * @param start     The start state
             * @param goal      The goal state
             * @return          Eigen::Vector3d 
             */
            Eigen::Vector3d get_diff(const core_datastructures::Posture& start, const core_datastructures::Posture& goal) const;
            
            /**
             * @brief checks if qiven vector is less than threhold
             * 
             * @param q_delta       The error difference for goal state
             * @return              true if values less than of threhold
             * @return              false if values greater than the threshold
             */
            bool is_less_than_threshold(Eigen::Vector3d& q_delta) const;

        private:
            double P0;
            double P3;
            Eigen::Vector3d perturb_params; // {P1, P2, SG}
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