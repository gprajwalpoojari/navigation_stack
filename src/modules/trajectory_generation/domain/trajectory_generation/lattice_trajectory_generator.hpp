#ifndef TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP

#include <i_trajectory_generator.hpp>
#include <cubic_spline_generator.hpp>
#include <memory>

namespace trajectory_generation::trajectory_generation {
    /**
     * @brief LatticeTrajectoryGenerator Class for generating cubic splines given two postures
     * 
     */
    class LatticeTrajectoryGenerator : public ITrajectoryGenerator {
        public:

        /**
         * @brief The constructor for @p LatticeTrajectoryGenerator object
         * 
         * @param[in]   start   The start posture
         * @param[in]   goal    The goal posture
         * 
        */
        LatticeTrajectoryGenerator(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal);
        

        /**
         * @brief   Generates trajectory given a start vehicle location, goal location and an acceleration profile
         * 
         * @param[in]   start_dyn_posture   The current vehicle location with velocity and time
         * @param[in]   goal                The goal vehicle location
         * @param[in]   acceleration        The acceleration to be applied
        */
        std::vector<core_datastructures::DynamicPosture> generate_trajectory(
                                                        const core_datastructures::DynamicPosture& start_dyn_posture, 
                                                        const core_datastructures::Posture& goal,double acceleration);
        
        /**
         * @brief Applies a constant acceleration profile on the trajectory
         * 
         * @param[in]   spline          A vector of vehicle postures from start to goal
         * @param[in]   acceleration    The acceleration value
        */
        std::vector<core_datastructures::DynamicPosture> apply_constant_acceleration_profile(
                                                            std::vector<core_datastructures::Posture>& spline, 
                                                            double acceleration);

        
        private:
        std::shared_ptr<spline_generation::CubicSplineGenerator> spline_generator;
        double start_velocity;
        double start_time;
    };
}


#endif  /*TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP*/