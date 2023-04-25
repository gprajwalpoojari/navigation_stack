#ifndef CUBIC_SPLINE_GENERATOR_HPP
#define CUBIC_SPLINE_GENERATOR_HPP

#include <spline_generation/ISplineGenerator.hpp>

namespace trajectory_generation {
    class CubicSplineGenerator {
        public:
            /*
                Generate splines as per given end posture
                @param[in] core_datastructures::Posture posture
            */
            std::vector<core_datastructures::Posture> generate_splines(core_datastructures::Posture posture);
    };
}


#endif  /*CUBIC_SPLINE_GENERATOR_HPP*/