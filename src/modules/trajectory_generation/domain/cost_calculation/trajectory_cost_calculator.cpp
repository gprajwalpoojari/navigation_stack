# include <trajectory_cost_calculator.hpp>
#include <core_datastructures/point.hpp>
#include <math_utils/geometry.hpp>
#include <cmath>
#include <limits>

namespace trajectory_generation::cost_calculation {
    TrajectoryCostCalculator::TrajectoryCostCalculator(const std::vector<core_datastructures::Posture>& trajectory, 
                                                        double lane_width) : lane_center(lane_center),
                                                        lane_width(lane_width) {
        lane_center_cost = 0;
        lane_deviation_cost = 1;
        init_idx = 0;
    }

    double TrajectoryCostCalculator::get_cost() const {
        return get_static_cost() + get_dynamic_cost();
    }
    
    double TrajectoryCostCalculator::get_static_cost() const {
        return get_static_lane_cost() + get_static_obstacle_cost();
    }

    double TrajectoryCostCalculator::get_dynamic_cost() const {
        return 0;
    }

    void TrajectoryCostCalculator::set(const std::vector<core_datastructures::DynamicPosture>& trajectory) {
        this->trajectory = trajectory;
        init_idx = 0;
    }

    double TrajectoryCostCalculator::get_static_lane_cost() const {
        double lane_cost = 0;
        for (const auto& traj_pt : trajectory) {
            double traj_pt_lane_cost = lane_center_cost + lane_deviation_cost * 
                                                            compute_lateral_deviation(traj_pt);
            lane_cost += traj_pt_lane_cost;
        }
        return lane_cost;
    }

    double TrajectoryCostCalculator::compute_lateral_deviation(const core_datastructures::DynamicPosture& traj_pt) const {
        double max_deviation = lane_width / 2;
        double lateral_deviation = std::numeric_limits<int>::max();
        core_datastructures::Point p1{traj_pt.x, traj_pt.y};
        for (const auto& lane_pt : lane_center) {
            core_datastructures::Point p2{lane_pt.x, lane_pt.y};
            double deviation = common::math_utils::get_distance(p1, p2);
            if (deviation < lateral_deviation) {
                lateral_deviation = deviation;
            }
        }
        return (lateral_deviation > max_deviation) ? std::numeric_limits<int>::max() : lateral_deviation;
    }

    bool TrajectoryCostCalculator::is_approx(double value, double set_point, double threshold) const {
        return (std::abs(value - set_point) <= threshold);
    }

    double TrajectoryCostCalculator::get_static_obstacle_cost() const {
        return 0;
    }
}