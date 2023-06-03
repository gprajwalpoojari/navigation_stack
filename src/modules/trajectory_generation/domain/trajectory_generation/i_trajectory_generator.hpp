#ifndef TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__I_TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__I_TRAJECTORY_GENERATOR_HPP

#include <core_datastructures/dynamic_posture.hpp>
#include <vector>

namespace trajectory_generation::trajectory_generation {
    class ITrajectoryGenerator {
        public:
            virtual std::vector<core_datastructures::DynamicPosture> generate_trajectory(double acceleration) = 0;
    };
}


#endif  /*TRAJECTORY_GENERATION__TRAJECTORY_GENERATION__I_TRAJECTORY_GENERATOR_HPP*/