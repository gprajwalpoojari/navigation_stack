#ifndef TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR
#define TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR


#include <i_cost_calculator.hpp>
#include <core_datastructures/dynamic_posture.hpp>
#include <vector>

namespace trajectory_generation::cost_calculation {

class TrajectoryCostCalculator : public ICostCalculator {
    public:
    /**
     * @brief Construct a new Trajectory Cost Calculator object
     * 
     * @param trajectory 
     */
    TrajectoryCostCalculator(const std::vector<core_datastructures::DynamicPosture>& trajectory);

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

    private:
    std::vector<core_datastructures::DynamicPosture> trajectory;
};

}

#endif /*TRAJECTORY_GENERATION__COST_CALCULATION__TRAJECTORY_COST_CALCULATOR*/