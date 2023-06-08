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

    core_datastructures::DynamicPosture to_domain(const common_ros2::msg::DynamicPosture& dyn_posture){
        core_datastructures::DynamicPosture new_dyn_posture{dyn_posture.x, dyn_posture.y, dyn_posture.theta, 
                                                            dyn_posture.kappa, dyn_posture.v, dyn_posture.t};
        return new_dyn_posture;
    }

    common_ros2::msg::DynamicPosture to_ros2(const core_datastructures::DynamicPosture& dyn_posture){
        common_ros2::msg::DynamicPosture new_dyn_posture;
        new_dyn_posture.x = dyn_posture.x;
        new_dyn_posture.y = dyn_posture.y;
        new_dyn_posture.theta = dyn_posture.theta;
        new_dyn_posture.kappa = dyn_posture.kappa;
        new_dyn_posture.v = dyn_posture.v;
        new_dyn_posture.t = dyn_posture.t;
        return new_dyn_posture;
    }
}