#include <converters.hpp>

#include <core_datastructures/posture.hpp>

namespace converters{
    core_datastructures::Posture to_domain(const common_ros2::msg::Posture& posture){
        core_datastructures::Posture new_posture{posture.x, posture.y, posture.theta, posture.kappa};
        return new_posture;
    }

    common_ros2::msg::Posture to_ros2(const core_datastructures::Posture& posture){
        common_ros2::msg::Posture new_posture;
        new_posture.x = posture.x;
        new_posture.y = posture.y;
        new_posture.theta = posture.theta;
        new_posture.kappa = posture.kappa;
        return new_posture;
    }
}