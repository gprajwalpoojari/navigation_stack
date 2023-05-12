#include <cubic_spline_generator.hpp>
#include <core_datastructures/Posture.hpp>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>


namespace trajectory_generation{
    CubicSplineGenerator::CubicSplineGenerator(){}

    CubicSplineGenerator::CubicSplineGenerator(core_datastructures::Posture& start, core_datastructures::Posture& goal){
        this->start = start;
        this->goal = goal;
        
        P0 = start.kappa;
        P3 = goal.kappa;
        SG = get_delta(start, goal)*20;

        P1=0.1;
        P2=0.1;
        q_thresh(0) =0.01;
        q_thresh(1) = 0.01;
        q_thresh(2)=0.01;
    }


    std::vector<core_datastructures::Posture> CubicSplineGenerator::get_spline(){
        run_gradient_descent();   
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
    
    std::vector<std::vector<double>> CubicSplineGenerator::calculate_Jacobian(std::vector<double>& params, double s){
        // P1 : 0
        // P2 : 1
        // SG : 2
        std::vector<std::vector<double>> J(3,std::vector<double>(3,0));
        core_datastructures::Posture state = get_next_state(params,s);
        std::vector<double> values  {P1, P2, s};
        std::cout << P1 << ", " << P2 << ", " << s << std::endl;
        // std::cout << state.x << ", " << state.y << ", " << state.theta << ", " << state.kappa << std::endl;
        double delta =0.1;
        for(int j=0; j<J.size(); j++){
            int t=0;
            // for(int k=0; k<delta*SG*1000;k++){
            values[j] +=delta;
            std::vector<double> new_param = calculate_parameters(values[0],values[1],values[2]);
            core_datastructures::Posture next_state = get_next_state(new_param,s);
            
            J[0][j] = (next_state.x - state.x)/delta;
            J[1][j] = (next_state.y - state.y)/delta;
            J[2][j] = (next_state.theta - state.theta)/delta;
            //     t++;
            // }
            values[j] -=delta;
            // *t;
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


    double CubicSplineGenerator::get_delta(const core_datastructures::Posture& d1, const core_datastructures::Posture& d2){
        return sqrt(pow(d2.y-d2.y,2) + pow(d2.x-d1.x,2));
    }

    Eigen::Vector3d CubicSplineGenerator::get_d(const core_datastructures::Posture& d1, const core_datastructures::Posture& d2){
        Eigen::Vector3d v(d2.x-d1.x, d2.y-d1.y, d2.theta-d1.theta);
        return v;
    }

    bool CubicSplineGenerator::comp(Eigen::Vector3d& v1, Eigen::Vector3d& v2){
        if(abs(v1[0])<v2[0] && abs(v1[1])<v2[1] && abs(v1[2])<v2[2])return true;
        return false;
    }

    Eigen::Matrix3d get_Jinv(std::vector<std::vector<double>>& J){
        Eigen::Matrix3d m;
        for(int i=0; i<J.size();i++){
            for(int j=0; j<J.size(); j++){
                m(i,j) = J[i][j];
            }
        }

        // return m.inverse();
        return m.completeOrthogonalDecomposition().pseudoInverse();
    }
    void CubicSplineGenerator::run_gradient_descent(){
        std::vector<double> new_params = calculate_parameters(P1,P2,SG);
        core_datastructures::Posture estimate_goal = get_next_state(new_params,SG);
        Eigen::Vector3d q_delta = get_d(estimate_goal,goal);
        Eigen::Vector3d phat(P1,P2,SG),p_(0,0,0);


        // std::cout<<"Estimate:goal"<<estimate_goal.x<<","<<estimate_goal.y<<std::endl;
        // std::cout<<q_delta<<std::endl;
        // std::cout<<phat<<std::endl;
        int iter=0;

        while(!comp(q_delta , q_thresh) && iter++<1000){
            // std::cout << q_delta << " , " << q_thresh << std::endl;
            std::cout<<iter<<std::endl;
            new_params = calculate_parameters(P1,P2,SG);
            estimate_goal = get_next_state(new_params,SG);
            std::vector<std::vector<double>> J = calculate_Jacobian(new_params,SG);
            q_delta = get_d(estimate_goal, goal);
            Eigen::Vector3d delta_p = get_Jinv(J)*q_delta;//Jinverse;
            // std::cout<< delta_p <<std::endl;
            p_ = phat+delta_p; //phat + delta_p
            P1 = p_(0);
            P2=p_(1);
            SG=p_(2);
            // std::cout<<estimate_goal.x<<","<<estimate_goal.y<<":"<<goal.x<<","<<goal.y<<std::endl;
            // std::cout<<"---------"<<P1<<","<<P2<<","<<SG<<std::endl;
            // std::cout<<p_<<std::endl;
        }
        // std::cout << "#################################################3" <<std::endl;
        // std::cout << q_delta << " , " << q_thresh << std::endl;
        // std::cout << "#################################################3" <<std::endl;
        // phat = p_;
        

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

// int main(){
//     core_datastructures::Posture start{0,0,0,0},goal{1,1,1.7,0};

//     std::vector<core_datastructures::Posture> trajectory_generation::CubicSplineGenerator::get_spline(start, goal);

//     return 0;
// }

