#ifndef COMMON__MATH_UTILS__GEOMETRY_HPP
#define COMMON__MATH_UTILS__GEOMETRY_HPP

#include <posture.hpp>
#include <utility>
#include <point.hpp>


namespace common::math_utils {


    /**
     * @brief Gets the euclidian distance given two points
     * 
     * @param start     The start point
     * @param goal      The goal point
     * @return          double eucilidian distance
     */
    double get_distance(const core_datastructures::Point& start, const core_datastructures::Point& goal);
    /**
     * @brief Gets the euclidian distance given two postures
     * 
     * @param start     The start state
     * @param goal      The goal state
     * @return          double eucilidian distance
     */
    double get_distance(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal);
    

    /**
     * @brief   Solves the quadratic equation a*x^2 + b*x + c = 0 for a given a, b, c
     *          Returns {0, 0} if roots are imaginary
     * 
     * @param[in]   a   double
     * @param[in]   b   double
     * @param[in]   c   double
     * 
     * @return  std::pair<double, double>   the roots of the quadratic equation. 
    */
    std::pair<double, double> solve_quadratic_equation(double a, double b, double c);

    /**
     * @brief Computes the slope of a line given two endpoints
     * 
     * @param[in] p1    Point 1
     * @param[in] p2    Point 2
     * @return double 
     */
    double compute_slope(const core_datastructures::Point& p1, const core_datastructures::Point& p2);
}

#endif  /*COMMON__MATH_UTILS__GEOMETRY_HPP*/