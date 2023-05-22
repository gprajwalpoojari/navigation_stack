#inlcude "converters.hpp"

#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/spline.hpp>
#include <core_datastructures/Point.hpp>
#include <core_datastructures/Posture.hpp>
#include <core_datastructures/Pose.hpp>

namespace convert{
    core_datastructures::Posture to_domain(const common_ros2::msg::Posture& posture){
        core_datastructures::Posture new_posture(posture.x, posture.y, posture.theta, posture.kappa);
        return new_posture;
    }

    common_ros2::msg::Possture to_ros2(const core_datastructures::Posture& posture){
        common_ros2::msg::Posture new_posture(posture.x, posture.y, posture.theta, posture.kappa);
        return new_posture;
    }
}