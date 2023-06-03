#ifndef CONVERTERS__CONVERTERS_HPP
#define CONVERTERS__CONVERTERS_HPP

#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/spline.hpp>
// #include <core_datastructures/Point.hpp>
#include <core_datastructures/Posture.hpp>
// #include <core_datastructures/Pose.hpp>

namespace converters{
    /**
     * @brief Convert ros2 Posture to domain
     * 
     * @param posture 
     * @return core_datastructures::Posture 
     */
    core_datastructures::Posture to_domain(const common_ros2::msg::Posture& posture);


    /**
     * @brief Convert domain Posture to ros2
     * 
     * @param posture
     * @return common_ros2::msg::Posture 
     */
    common_ros2::msg::Posture to_ros2(const core_datastructures::Posture& posture);
}

#endif // CONVERTERS__CONVERTERS_HPP