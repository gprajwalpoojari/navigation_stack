#include <cubic_spline_generator.hpp>
#include <core_datastructures/Posture.hpp>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>


namespace trajectory_generation{

    double get_delta(const core_datastructures::Posture& start, const core_datastructures::Posture& goal){
        return sqrt(pow(goal.y-start.y,2) + pow(goal.x-start.x,2));
    }

    Eigen::Vector3d get_d(const core_datastructures::Posture& start, const core_datastructures::Posture& goal){
        Eigen::Vector3d v(goal.x-start.x, goal.y-start.y, goal.theta-start.theta);
        return v;
    }

    bool comp(Eigen::Vector3d& v1, Eigen::Vector3d& v2){
        if(v1[0]<v2[0] && v1[1]<v2[1] && v1[2]<v2[2])return true;
        return false;
    }

    CubicSplineGenerator::CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal){
        this->start = start;
        this->goal = goal;
        
        P0 = start.kappa;
        P3 = goal.kappa;
        SG = get_delta(start, goal);

        P1=0.1;
        P2=0.1;
        q_thresh(0) =0.01;
        q_thresh(1) = 0.01;
        q_thresh(2)=0.01;
    }


    std::vector<core_datastructures::Posture> CubicSplineGenerator::get_spline(){
        run_gradient_descent();   
        std::cout << P1 << " " << P2 << std::endl;
        return generate_splines();
    }

    std::vector<double> CubicSplineGenerator::calculate_parameters(double p1, double p2, double s){
        double a = P0;
        double b = -(11*P0 - 18*p1 + 9*p2 - 2*P3)/(2*s);
        double c = 9*(2*P0 -5*p1 + 4*p2 - P3)/(2*pow(s,2));
        double d = -9*(P0 -3*p1 + 3*p2 -P3)/(2*pow(s,3));

        return {a,b,c,d};
    }

    core_datastructures::Posture CubicSplineGenerator::get_next_state(std::vector<double>& params, double s){
        core_datastructures::Posture next_state;
        next_state.kappa = calculate_kappa(params,s);
        next_state.theta = calculate_theta(params,s);
        next_state.x = calculate_x(params,s);
        next_state.y = calculate_y(params,s);

        return next_state;
    }

    double CubicSplineGenerator::calculate_kappa(std::vector<double>& params, double s){
        return params[0] + params[1]*s + params[2]*pow(s,2) + params[3]*pow(s,3);
    }

    double CubicSplineGenerator::calculate_theta(std::vector<double>& params, double s){
        return params[0]*s + params[1]*pow(s,2)/2 + params[2]*pow(s,3)/3 + params[3]*pow(s,4)/4;
    }
    //calculate Jacobian
    
    Eigen::Matrix3d CubicSplineGenerator::calculate_Jacobian(std::vector<double>& params){
        // P1 : 0
        // P2 : 1
        // SG : 2
        Eigen::Matrix3d J;
        core_datastructures::Posture state = get_next_state(params, SG);
        std::vector<double> values  {P1, P2, SG};
        double delta = 0.1;
        std::vector<double> perturb {delta, delta, delta};
        std::vector<double> param = calculate_parameters(values[0],values[1],values[2]);
        for(int j=0; j<J.rows(); j++){
            values[j] +=perturb[j];

            std::vector<double> new_param = calculate_parameters(values[0],values[1],values[2]);
            // if (j == 2) {
            //     std::cout << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << std::endl;
            //     std::cout << new_param[0] << " " << new_param[1] << " " << new_param[2] << " " << new_param[3] << std::endl;
            // }
            core_datastructures::Posture next_state = get_next_state(new_param, values[2]);
            
            J(0, j) = (next_state.x - state.x)/perturb[j];
            J(1, j) = (next_state.y - state.y)/perturb[j];
            J(2, j) = (next_state.theta - state.theta)/perturb[j];

            values[j] -= perturb[j];
        }
        return J;
    }


    double CubicSplineGenerator::calculate_x(std::vector<double>& params, double s){
        double prev_theta = calculate_theta(params, 0);
        double prev_x = cos(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=s; i+=0.1){
            double curr_theta = calculate_theta(params,i);
            double curr_x = cos(curr_theta);
            total_area += (prev_x+curr_x)*0.5*0.1;
            prev_x=curr_x;
        }

        return total_area;
    }

    double CubicSplineGenerator::calculate_y(std::vector<double>& params, double s){
        double prev_theta = calculate_theta(params, 0);
        double prev_y = sin(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=s; i+=0.1){
            double curr_theta = calculate_theta(params,i);
            double curr_y = sin(curr_theta);
            total_area += (prev_y+curr_y)*0.5*0.1;
            prev_y=curr_y;
        }

        return total_area;
    }

    void CubicSplineGenerator::run_gradient_descent(){
        Eigen::Vector3d q_delta(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), 
                                std::numeric_limits<int>::max());
        Eigen::Vector3d phat(P1,P2,SG);
        int iter = 0;
        // while ((delta_p[0] > 1e-2 || delta_p[1] > 1e-2 || delta_p[2] > 1e-4) && iter < 1000) {
        while((q_delta[0] > q_thresh[0] || q_delta[1] > q_thresh[1] || q_delta[2] > q_thresh[2]) && iter < 1000){
            iter++;
            std::vector<double> params = calculate_parameters(phat[0], phat[1], phat[2]);
            core_datastructures::Posture estimate_goal = get_next_state(params,phat[2]);
            Eigen::Matrix3d J = calculate_Jacobian(params);
            // if (iter < 10) {
            //     std::cout << iter << std::endl;
            //     std::cout << J.completeOrthogonalDecomposition().pseudoInverse() << std::endl;
            //     std::cout << std::endl;
            // }

            q_delta = get_d(estimate_goal, goal);
            Eigen::Vector3d delta_p = J.completeOrthogonalDecomposition().pseudoInverse() * q_delta;//Jinverse;
            
            
            // std::cout << std::endl;
            phat += delta_p; //phat + delta_p
            P1 = phat[0];
            P2 = phat[1];
            SG = phat[2];
            // std::cout << params[0] << " " << params[1] << " " << params[2] << " " << params[3] << std::endl;
            // std::cout << phat[0] << " " << phat[1] << " " << phat[2] << std::endl;
            // std::cout << delta_p[0] << " " << delta_p[1] << " " << delta_p[2] << std::endl;

            // std::cout << q_thresh[0] << " " << q_thresh[1] << " " << q_thresh[2] << std::endl;

            // std::cout << estimate_goal.x << " " << estimate_goal.y << " " << estimate_goal.theta << std::endl;
            // std::cout << goal.x << " " << goal.y << " " << goal.theta << std::endl;
            // std::cout << q_delta[0] << " " << q_delta[1] << " " << q_delta[2] << std::endl;
        }
        
    }

    std::vector<core_datastructures::Posture> CubicSplineGenerator::generate_splines(){
        std::vector<core_datastructures::Posture> spline_points;
        std::vector<double> params = calculate_parameters(P1,P2,SG);
        
        for(double i=0; i<=SG; i+=0.1){
            core_datastructures::Posture state = get_next_state(params, i);
            spline_points.push_back(state);
        }
        return spline_points;
    }

}

