#ifndef TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR
#define TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR


#include <i_cost_calculator.hpp>
#include <core_datastructures/dynamic_posture.hpp>
#include <core_datastructures/posture.hpp>
#include <vector>

namespace trajectory_generation::cost_calculation {

class TrajectoryCostCalculator : public ICostCalculator {
    public:
    /**
     * @brief Construct a new Trajectory Cost Calculator object
     * 
     * @param lane_center The lane center 
     */
    TrajectoryCostCalculator(const std::vector<core_datastructures::Posture>& lane_center, double lane_width);

    /**
     * @brief Get the sum of static and dynamic cost
     * 
     * @return double 
     */
    double get_cost() const;

    /**
     * @brief Get the static cost
     * 
     * @return double 
     */
    double get_static_cost() const;

    /**
     * @brief Get the dynamic cost
     * 
     * @return double 
     */
    double get_dynamic_cost() const;

    /**
     * @brief Set the trajectory for computing cost
     * 
     * @param[in] trajectory The trajectory to be set 
     * @return void 
     */
    void set(const std::vector<core_datastructures::DynamicPosture>& trajectory);

    /**
     * @brief Get the static lane cost for the trajectory
     * 
     * @return double 
     */
    double get_static_lane_cost() const;

    /**
     * @brief Computes the lateral deviation from the road center line
     * 
     * @param trajectory_point The trajectory point 
     * @return double 
     */
    double compute_lateral_deviation(const core_datastructures::DynamicPosture& trajectory_point) const;

    /**
     * @brief Checkes if the input value is approximately equal to the given setpoint
     * 
     * @param[in] value 
     * @param[in] set_point 
     * @param[in] threshold 
     * @return true if difference between @p value and @p set_point is less than threshold
     * @return false if difference between @p value and @p set_point is greater than threshold
     */
    bool is_approx(double value, double set_point, double threshold=0.1) const;

    /**
     * @brief Get the static obstacle cost
     * 
     * @return double 
     */
    double get_static_obstacle_cost() const;

    private:
    std::vector<core_datastructures::Posture> lane_center;
    std::vector<core_datastructures::DynamicPosture> trajectory;
    double lane_center_cost;
    double lane_deviation_cost;
    double lane_width;
    int init_idx;
};

}

#endif /*TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR*/