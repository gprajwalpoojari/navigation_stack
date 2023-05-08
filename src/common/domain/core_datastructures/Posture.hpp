#ifndef CORE_DATASTRUCTURES__POSTURE_HPP
#define CORE_DATASTRUCTURES__POSTURE_HPP

namespace core_datastructures {
    struct Posture {
        double x;       // x position
        double y;       // y position
        double theta;   // orientation
        double kappa;   // curvature
    };
}

#endif  /*CORE_DATASTRUCTURES__POSTURE_HPP*/