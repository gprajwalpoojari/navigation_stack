#ifndef TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP

#include <i_trajectory_generator.hpp>
#include <i_spline_generator.hpp>

namespace trajectory_generation::trajectory_generation {
    /**
     * @brief LatticeTrajectoryGenerator Class for generating cubic splines given two postures
     * 
     */
    class LatticeTrajectoryGenerator : public ITrajectoryGenerator {
        public:
        TrajectoryGenerator(std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator> spline_generator);
        
        std::vector<core_datastructures::DynamicPosture> generate_trajectory(double acceleration);
        
        
        private:
        std::shared_ptr<trajectory_generation::spline_generation::ISplineGenerator> spline_generator;
        double start_velocity;
        double start_time;
        


    };
}


#endif  /*TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__LATTICE_TRAJECTORY_GENERATOR_HPP*/