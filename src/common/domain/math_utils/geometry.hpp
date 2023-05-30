#ifndef COMMON__MATH_UTILS__GEOMETRY_HPP
#define COMMON__MATH_UTILS__GEOMETRY_HPP

#include <Posture.hpp>


namespace common::math_utils {
    /**
     * @brief Gets the euclidian distance given two postures
     * 
     * @param start     The start state
     * @param goal      The goal state
     * @return          double eucilidian distance
     */
    double get_distance(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal);
}

#endif  /*COMMON__MATH_UTILS__GEOMETRY_HPP*/