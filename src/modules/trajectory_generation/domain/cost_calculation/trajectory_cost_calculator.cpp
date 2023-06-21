# include <trajectory_cost_calculator.hpp>


namespace trajectory_generation::cost_calculation {
    TrajectoryCostCalculator::TrajectoryCostCalculator(const std::vector<core_datastructures::DynamicPosture>& trajectory) : trajectory(trajectory) {

    }

    double TrajectoryCostCalculator::get_cost() const {
        return get_static_cost() + get_dynamic_cost();
    }
    
    double TrajectoryCostCalculator::get_static_cost() const {
        return 0;
    }

    double TrajectoryCostCalculator::get_dynamic_cost() const {
        return 0;
    }

    void TrajectoryCostCalculator::set(const std::vector<core_datastructures::DynamicPosture>& trajectory) {
        this->trajectory = trajectory;
    }
}