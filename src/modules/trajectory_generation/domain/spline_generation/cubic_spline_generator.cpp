#include <spline_generation/cubic_spline_generator.hpp>
#include <core_datastructures/Posture.hpp>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>


namespace trajectory_generation{
    CubicSplineGenerator::CubicSplineGenerator(){
        std::cout<<"DEFINED CONSTRUCTOR" <<std::endl;
    }
    CubicSplineGenerator::CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal){
        this->start = start;
        this->goal = goal;
        
        P0 = start.kappa;
        P3 = goal.kappa;
        SG = get_delta(start, goal);

        P1=1;
        P2=1;
        q_thresh(0) =0.1;
        q_thresh(1) = 0.1;
        q_thresh(2)=0.1;
    }


    std::vector<core_datastructures::Posture> CubicSplineGenerator::get_spline(){
        run_gradient_descent();   
        
        return generate_spline();
    }

    std::vector<double> CubicSplineGenerator::calculate_parameters(double P1, double P2, double P3){
        double a = P0;
        double b = -(11*P0 - 18*P1 + 9*P2 - 2*P3)/(2*SG);
        double c = 9*(2*P0 -5*P1 + 4*P2 - P3)/(2*pow(SG,2));
        double d = -9*(P0 -3*P1 + 3*P2 -P3)/(2*pow(SG,3));

        return {a,b,c,d};
    }

    core_datastructures::Posture CubicSplineGenerator::get_next_state(std::vector<double>& params, double SG){
        core_datastructures::Posture next_state;
        next_state.kappa = calculate_kappa(params,SG);
        next_state.theta = calculate_theta(params,SG);
        next_state.x = calculate_x(params,SG);
        next_state.y = calculate_y(params,SG);

        return next_state;
    }

    double CubicSplineGenerator::calculate_kappa(std::vector<double>& params, double SG){
        return params[0] + params[1]*SG + params[2]*pow(SG,2) + params[3]*pow(SG,3);
    }

    double CubicSplineGenerator::calculate_theta(std::vector<double>& params, double SG){
        return params[0]*SG + params[1]*pow(SG,2)/2 + params[2]*pow(SG,3)/3 + params[3]*pow(SG,4)/4;
    }
    //calculate Jacobian
    
    std::vector<std::vector<double>> CubicSplineGenerator::calculate_Jacobian(std::vector<double>& params, double SG){
        // P1 : 0
        // P2 : 1
        // SG : 2
        std::vector<std::vector<double>> J(3,std::vector<double>(3,0));
        core_datastructures::Posture state = get_next_state(params,SG);
        std::vector<double> values  {P1, P2, SG};
        double delta =0.1;
        for(int j=0; j<J.size(); j++){
            values[j] +=delta;
            std::vector<double> new_param = calculate_parameters(values[0],values[1],values[2]);
            core_datastructures::Posture next_state = get_next_state(new_param,SG);
            
            J[0][j] = (next_state.x - state.x)/delta;
            J[1][j] = (next_state.y - state.y)/delta;
            J[2][j] = (next_state.theta - state.theta)/delta;

            values[j] -=delta;
        }


        return J;
    }


    double CubicSplineGenerator::calculate_x(std::vector<double>& params, double SG){
        double prev_theta = calculate_theta(params, 0);
        double prev_x = cos(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=SG; i+=0.1){
            double curr_theta = calculate_theta(params,i);
            double curr_x = cos(curr_theta);
            total_area += (prev_x+curr_x)*0.5*0.1;
            prev_x=curr_x;
        }

        return total_area;
    }

    double CubicSplineGenerator::calculate_y(std::vector<double>& params, double SG){
        double prev_theta = calculate_theta(params, 0);
        double prev_y = sin(prev_theta);
        double total_area{0};
        for(double i=0.1; i<=SG; i+=0.1){
            double curr_theta = calculate_theta(params,i);
            double curr_y = sin(curr_theta);
            total_area += (prev_y+curr_y)*0.5*0.1;
            prev_y=curr_y;
        }

        return total_area;
    }


    double get_delta(const core_datastructures::Posture& start, const core_datastructures::Posture& goal){
        return sqrt(pow(goal.y-start.y,2) + pow(goal.x-start.x,2));
    }

    Eigen::Vector3d get_d(const core_datastructures::Posture& start, const core_datastructures::Posture& goal){
        Eigen::Vector3d v(goal.x-start.x, goal.y-start.y, goal.theta-start.theta);
        return v;
    }

    bool comp(Eigen::Vector3d& v1, Eigen::Vector3d& v2){
        if(v1[0]>v2[0] && v1[1]>v2[1] && v1[2]>v2[2])return true;
        return false;
    }

    Eigen::Matrix3d get_Jinv(std::vector<std::vector<double>>& J){
        Eigen::Matrix3d m;
        for(int i=0; i<J.size();i++){
            for(int j=0; j<J.size(); j++){
                m(i,j) = J[i][j];
            }
        }

        return m.inverse();
    }
    void CubicSplineGenerator::run_gradient_descent(){
        // double q_delta = get_delta(start, goal); // vectorize this ~~~
        Eigen::Vector3d q_delta = get_d(start,goal);
        Eigen::Vector3d phat(P1,P2,SG),p_;

        while(comp(q_delta , q_thresh)){
            std::vector<double> new_params = calculate_parameters(P1,P2,SG);
            core_datastructures::Posture estimate_goal = get_next_state(new_params,SG);
            std::vector<std::vector<double>> J = calculate_Jacobian(new_params,SG);
            q_delta = get_d(estimate_goal, goal);
            Eigen::Vector3d delta_p = get_Jinv(J)*q_delta;//Jinverse;
            p_ = delta_p + phat; //phat + delta_p
        }
        phat = p_;

    }

    std::vector<core_datastructures::Posture> CubicSplineGenerator::generate_spline(){
        std::vector<core_datastructures::Posture> spline_points;
        std::vector<double> params = calculate_parameters(P1,P2,SG);
        
        for(double i=0; i<=SG; i+=0.1){
            core_datastructures::Posture state = get_next_state(params, i);
            spline_points.push_back(state);
        }
        return spline_points;
    }

}

// int main(){
//     core_datastructures::Posture start{0,0,0,0},goal{1,1,1.7,0};

//     std::vector<core_datastructures::Posture> trajectory_generation::CubicSplineGenerator::get_spline(start, goal);

//     return 0;
// }


namespace trajectory_generation{
    CubicSplineGenerator::CubicSplineGenerator(const core_datastructures::Posture& start, const core_datastructures::Posture& goal) : start(start), goal(goal) {
    
    }
    std::vector<core_datastructures::Posture> CubicSplineGenerator::generate_splines(core_datastructures::Posture posture) {
        return std::vector<core_datastructures::Posture>{};
    }
}