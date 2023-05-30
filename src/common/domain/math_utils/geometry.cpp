#include <geometry.hpp>
#include <cmath>

namespace common::math_utils {
    double get_distance(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal) {
        return std::sqrt(std::pow(goal.x - start.x,2) + std::pow(goal.y - start.y, 2));
    }
}