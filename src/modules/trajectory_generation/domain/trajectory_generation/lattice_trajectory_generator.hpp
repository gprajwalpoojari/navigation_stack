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
        LatticeTrajectoryGenerator(const core_datastructures::Posture& start, 
                                                const core_datastructures::Posture& goal);
        
        std::vector<core_datastructures::DynamicPosture> generate_trajectory(const core_datastructures::DynamicPosture& start_dyn_posture, 
                                                            const core_datastructures::Posture& goal,double acceleration);
        
        std::vector<core_datastructures::DynamicPosture> apply_acceleration(std::vector<core_datastructures::Posture>& spline, double acceleration);

        
        private:
        std::shared_ptr<spline_generation::CubicSplineGenerator> spline_generator;
        double start_velocity;
        double start_time;
        


    };
}


#endif  /*TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP*/