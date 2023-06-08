#ifndef TRAJECTORY_GENERATION__SPLINE_GENERATION__I_SPLINE_GENERATOR_HPP
#define TRAJECTORY_GENERATION__SPLINE_GENERATION__I_SPLINE_GENERATOR_HPP

#include <core_datastructures/posture.hpp>
#include <vector>

namespace trajectory_generation::spline_generation {
    class ISplineGenerator {
        public:
            virtual std::vector<core_datastructures::Posture> get_spline(const core_datastructures::Posture& start, 
                                                                         const core_datastructures::Posture& goal) = 0;
    };
}


#endif  /*TRAJECTORY_GENERATION__SPLINE_GENERATION__I_SPLINE_GENERATOR_HPP*/