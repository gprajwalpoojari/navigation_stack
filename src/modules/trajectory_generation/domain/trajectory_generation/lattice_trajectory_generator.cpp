#include <lattice_trajectory_generator.hpp>
#include <cmath>
#include <math_utils/geometry.hpp>
#include <iostream>
#include <stdexcept>

namespace trajectory_generation::trajectory_generation{
    LatticeTrajectoryGenerator::LatticeTrajectoryGenerator(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal) {
        
        start_velocity = 0;
        start_time = 0;
        spline_generator = std::make_shared<spline_generation::CubicSplineGenerator>(start, goal);
    }

    std::vector<core_datastructures::DynamicPosture> LatticeTrajectoryGenerator::apply_constant_acceleration_profile(
                                                                                std::vector<core_datastructures::Posture>& spline, 
                                                                                double acceleration) {
        std::vector<core_datastructures::DynamicPosture> trajectory;
        trajectory.push_back(core_datastructures::DynamicPosture{spline[0].x, spline[0].y, spline[0].theta, 
                                                                spline[0].kappa, start_velocity, start_time});
        if (start_velocity == 0 && acceleration == 0) {
            throw std::runtime_error("Cannot generate trajectory when start velocity and acceleration both are zeros");
        }
        double s = 0;
        for (int i = 1; i < spline.size(); i++) {
            double delta_s = common::math_utils::get_distance(spline[i-1], spline[i]);
            s += delta_s;
            // v^2 = u^2 + 2*a*s
            double next_velocity = std::sqrt(std::pow(trajectory[i-1].v, 2) + 2 * acceleration * delta_s);
            // s = u*t + 0.5 * a * t^2
            auto pair = common::math_utils::solve_quadratic_equation(0.5*acceleration, trajectory[i-1].v, -s);
            double delta_t = std::max({0.0, pair.first, pair.second});
            double next_time = start_time + delta_t;
            trajectory.push_back(core_datastructures::DynamicPosture{spline[i].x, spline[i].y, spline[i].theta, 
                                                                        spline[i].kappa, next_velocity, next_time});
            std::cout << next_velocity <<  " " << next_time << std::endl;
        }
        return trajectory;
    }



    std::vector<core_datastructures::DynamicPosture> LatticeTrajectoryGenerator::generate_trajectory(
                                                            const core_datastructures::DynamicPosture& start_dyn_posture, 
                                                            const core_datastructures::Posture& goal, double acceleration)
    {
        start_velocity = start_dyn_posture.v;
        start_time = start_dyn_posture.t;
        
        core_datastructures::Posture start{start_dyn_posture.x, start_dyn_posture.y,start_dyn_posture.theta, start_dyn_posture.kappa};
        std::vector<core_datastructures::Posture> spline = spline_generator->get_spline(start, goal);
        std::vector<core_datastructures::DynamicPosture> trajectory = apply_constant_acceleration_profile(spline, acceleration);
        return trajectory;
    }

}

