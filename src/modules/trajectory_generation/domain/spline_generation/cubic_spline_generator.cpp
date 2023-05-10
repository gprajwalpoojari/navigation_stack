#include <cubic_spline_generator.hpp>

namespace trajectory_generation{
    CubicSplineGenerator::CubicSplineGenerator(const core_datastructures::Posture& start, const core_datastructures::Posture& goal) : start(start), goal(goal) {
    
    }
    std::vector<core_datastructures::Posture> CubicSplineGenerator::generate_splines(core_datastructures::Posture posture) {
        return std::vector<core_datastructures::Posture>{};
    }
}