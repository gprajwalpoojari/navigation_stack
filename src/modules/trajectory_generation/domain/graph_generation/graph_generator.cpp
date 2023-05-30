#include <graph_generator.hpp>
#include <math_utils/geometry.hpp>
#include <cmath>
#include <iostream>

namespace trajectory_generation::graph_generation {
    
    GraphGenerator::GraphGenerator(std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator> spline_generator, 
                                    const std::vector<core_datastructures::Posture>& road_center, 
                                    double long_sampling_dist, double lane_width) 
                                    :road_center(road_center), a_s(long_sampling_dist), a_l(-(lane_width/2.0)) {
        
        downsample();
        this->spline_generator = std::move(spline_generator);
        h_max = this->road_center.size();
        i_max = 5;
        b_l = lane_width / i_max;
    }

    double GraphGenerator::get_s(int h) {
        return a_s * h;
    }
    double GraphGenerator::get_l(int i) {
        return a_l + b_l * i;
    }

    const core_datastructures::Posture GraphGenerator::to_cartesian(int h, int i) {
        double s = get_s(h);
        double l = get_l(i);
        core_datastructures::Posture road_center_point = road_center[h];
        core_datastructures::Posture frenet_to_cart;
        frenet_to_cart.x = road_center_point.x + l * cos(road_center_point.theta + (M_PI / 2));
        frenet_to_cart.y = road_center_point.y + l * sin(road_center_point.theta + (M_PI / 2));
        frenet_to_cart.theta = road_center_point.theta;
        frenet_to_cart.kappa = 1 / (1 / ((std::abs(road_center_point.kappa) <= 1e-4) ? 1e-4 : road_center_point.kappa) - l);
        return frenet_to_cart;
    }


    void GraphGenerator::downsample() {
        std::vector<core_datastructures::Posture> downsampled_road_center;
        downsampled_road_center.push_back(road_center[0]);
        double arc_length = 0;
        for (int i = 1; i < road_center.size(); i++) {
            arc_length += common::math_utils::get_distance(road_center[i], road_center[i-1]);
            if (arc_length >= a_s) {
                downsampled_road_center.push_back(road_center[i]);
                arc_length = 0;
            }
        }
        road_center = downsampled_road_center;
    }

    void GraphGenerator::search_graph() {
            for (int h = 0; h < h_max; h++) {
                for (int i = 0; i < i_max; i++) {
                    core_datastructures::Posture start = to_cartesian(h, i);
                    if (h==0) {
                        start = to_cartesian(h, 2);
                    }
                    for (int h_prime = h + 1; h_prime < h_max; h_prime++) {
                        for (int i_prime = 0; i_prime < i_max; i_prime++) {
                            core_datastructures::Posture goal = to_cartesian(h_prime, i_prime);
                            splines.push_back(spline_generator->get_spline(start, goal));
                        }
                    }
                    if (h==0) {
                        break;
                    }
                }
            }
    }

}
