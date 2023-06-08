#ifndef CORE_DATASTRUCTURES__DYNAMIC_POSTURE_HPP
#define CORE_DATASTRUCTURES__DYNAMIC_POSTURE_HPP

namespace core_datastructures {
    struct DynamicPosture {
        double x;       // x position
        double y;       // y position
        double theta;   // orientation
        double kappa;   // curvature
        double v;       // longitudinal velocity
        double t;       // time
    };
}

#endif  /*CORE_DATASTRUCTURES__DYNAMIC_POSTURE_HPP*/