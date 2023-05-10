#ifndef I_SPLINE_GENERATOR_HPP
#define I_SPLINE_GENERATOR_HPP

#include <core_datastructures/Posture.hpp>
#include <vector>

namespace trajectory_generation {
    class ISplineGenerator {
        public:
            virtual std::vector<core_datastructures::Posture> generate_spline() = 0;
    };
}


#endif  /*I_SPLINE_GENERATOR_HPP*/