#ifndef TRAJECTORY_GENERATION__COST_CALCULATION__I_COST_CALCULATOR_HPP
#define TRAJECTORY_GENERATION__COST_CALCULATION__I_COST_CALCULATOR_HPP


namespace trajectory_generation::cost_calculation {
    
    class ICostCalculator {
        public:
        /**
         * @brief Get the overall cost for an object
         * 
         * @return double 
         */
        virtual double get_cost() const = 0;
    };
}

#endif /*TRAJECTORY_GENERATION__COST_CALCULATION__I_COST_CALCULATOR_HPP*/