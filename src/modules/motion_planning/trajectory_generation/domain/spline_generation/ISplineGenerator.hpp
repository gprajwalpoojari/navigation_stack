#ifndef I_SPLINE_GENERATOR_HPP
#define I_SPLINE_GENERATOR_HPP

namespace trajectory_generation {
    class ISplineGenerator {
        public:
            virtual void generate_spline() = 0;
    };
}


#endif  /*I_SPLINE_GENERATOR_HPP*/