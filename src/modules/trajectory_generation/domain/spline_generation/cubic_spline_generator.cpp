#include <cubic_spline_generator.hpp>
#include <core_datastructures/Posture.hpp>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>


namespace trajectory_generation{

    CubicSplineGenerator::CubicSplineGenerator(const core_datastructures::Posture& start, const core_datastructures::Posture& goal) {       
        this->goal.x = goal.x - start.x;
        this->goal.y = goal.y - start.y;
        this->goal.theta = goal.theta - start.theta;
        this->goal.kappa = goal.kappa - start.kappa;
        
        P0 = start.kappa;
        P3 = goal.kappa;

        perturb_params << 0.1, 0.1, get_distance(start, goal);
        q_thresh << 0.01, 0.01, 0.01;
    }


    std::vector<core_datastructures::Posture> CubicSplineGenerator::get_spline(){
        run_gradient_descent();   
        std::cout << "Final Spline Parmaeters : " << perturb_params[0] << " " << perturb_params[1] << std::endl;
        return generate_splines();
    }

    const std::vector<double> CubicSplineGenerator::calculate_spline_coefficients() const{
        double P1 = perturb_params[0];
        double P2 = perturb_params[1];
        double SG = perturb_params[2];
        double a = P0;
        double b = -(11*P0 - 18*P1 + 9*P2 - 2*P3)/(2*SG);
        double c = 9*(2*P0 -5*P1 + 4*P2 - P3)/(2*pow(SG,2));
        double d = -9*(P0 -3*P1 + 3*P2 -P3)/(2*pow(SG,3));

        return {a,b,c,d};
    }

    const core_datastructures::Posture CubicSplineGenerator::get_next_state(const std::vector<double>& coeffs, double s) const {
        core_datastructures::Posture next_state;
        next_state.kappa = calculate_kappa(coeffs,s);
        next_state.theta = calculate_theta(coeffs,s);
        next_state.x = calculate_x(coeffs,s);
        next_state.y = calculate_y(coeffs,s);

        return next_state;
    }

    double CubicSplineGenerator::calculate_kappa(const std::vector<double>& coeffs, double s) const {
        return coeffs[0] + coeffs[1]*s + coeffs[2]*pow(s,2) + coeffs[3]*pow(s,3);
    }

    double CubicSplineGenerator::calculate_theta(const std::vector<double>& coeffs, double s) const{
        return coeffs[0]*s + coeffs[1]*pow(s,2)/2 + coeffs[2]*pow(s,3)/3 + coeffs[3]*pow(s,4)/4;
    }
    
    Eigen::Matrix3d CubicSplineGenerator::calculate_Jacobian(const std::vector<double>& coeffs){
        Eigen::Matrix3d J;
        core_datastructures::Posture state = get_next_state(coeffs, perturb_params[2]);
        double delta = 0.1;
        for(int j=0; j<J.rows(); j++){
            perturb_params[j] += delta;
            std::vector<double> coeffs = calculate_spline_coefficients();
            core_datastructures::Posture next_state = get_next_state(coeffs, perturb_params[2]);
            
            J(0, j) = (next_state.x - state.x)/delta;
            J(1, j) = (next_state.y - state.y)/delta;
            J(2, j) = (next_state.theta - state.theta)/delta;

            perturb_params[j] -= delta;
        }
        return J;
    }


    double CubicSplineGenerator::calculate_x(const std::vector<double>& coeffs, double s) const {
        double prev_theta = calculate_theta(coeffs, 0);
        double prev_x = cos(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=s; i+=0.1){
            double curr_theta = calculate_theta(coeffs,i);
            double curr_x = cos(curr_theta);
            total_area += (prev_x+curr_x)*0.5*0.1;
            prev_x=curr_x;
        }
        return total_area;
    }

    double CubicSplineGenerator::calculate_y(const std::vector<double>& coeffs, double s) const {
        double prev_theta = calculate_theta(coeffs, 0);
        double prev_y = sin(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=s; i+=0.1){
            double curr_theta = calculate_theta(coeffs,i);
            double curr_y = sin(curr_theta);
            total_area += (prev_y+curr_y)*0.5*0.1;
            prev_y=curr_y;
        }
        return total_area;
    }

    void CubicSplineGenerator::run_gradient_descent(){
        Eigen::Vector3d q_delta(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), 
                                std::numeric_limits<int>::max());
        int iter = 0;
        while(!is_less_than_threshold(q_delta) && iter < 1000){
            iter++;
            std::vector<double> coeffs = calculate_spline_coefficients();
            core_datastructures::Posture estimate_goal = get_next_state(coeffs,perturb_params[2]);
            Eigen::Matrix3d J = calculate_Jacobian(coeffs);
            q_delta = get_diff(estimate_goal, goal);
            Eigen::Vector3d delta_p = J.completeOrthogonalDecomposition().pseudoInverse() * q_delta;
            perturb_params += delta_p;
        }
        
    }

    const std::vector<core_datastructures::Posture> CubicSplineGenerator::generate_splines() const{
        std::vector<core_datastructures::Posture> spline_points;
        std::vector<double> coeffs = calculate_spline_coefficients();
        
        for(double i=0; i<=perturb_params[2]; i+=0.1){
            core_datastructures::Posture state = get_next_state(coeffs, i);
            spline_points.push_back(state);
        }
        return spline_points;
    }

    Eigen::Vector3d CubicSplineGenerator::get_diff(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal) const{
        Eigen::Vector3d v(goal.x - start.x, goal.y - start.y, goal.theta - start.theta);
        return v;
    }

    bool CubicSplineGenerator::is_less_than_threshold(Eigen::Vector3d& q_delta) const{
        if(q_delta[0] > q_thresh[0] || q_delta[1] > q_thresh[1] || q_delta[2] > q_thresh[2]) {
            return false;
        }
        return true;
    }

    double CubicSplineGenerator::get_distance(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal) const{
        return std::sqrt(std::pow(goal.x - start.x,2) + std::pow(goal.y - start.y, 2));
    }

}

