#include <geometry.hpp>
#include <cmath>

namespace common::math_utils {
    double get_distance(const core_datastructures::Point& start, 
                                                const core_datastructures::Point& goal) {
        return std::sqrt(std::pow(goal.x - start.x,2) + std::pow(goal.y - start.y, 2));
    }

    double get_distance(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal) {
        return std::sqrt(std::pow(goal.x - start.x,2) + std::pow(goal.y - start.y, 2));
    }

    std::pair<double, double> solve_quadratic_equation(double a, double b, double c) {
        double radical = std::pow(b, 2) - 4 * a * c;
        if (radical < 0) {
            return std::make_pair(0, 0);
        }
        double root_1 = (-b + std::sqrt(radical)) / (2 * a);
        double root_2 = (-b - std::sqrt(radical)) / (2 * a);
        return std::make_pair(root_1, root_2);
    }

    double compute_slope(const core_datastructures::Point& p1, const core_datastructures::Point& p2) {
        return (p2.y - p1.y) / (p2.x - p1.x);
    }

}