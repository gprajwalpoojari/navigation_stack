#pragma once

#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/spline.hpp>
#include <core_datastructures/Point.hpp>
#include <core_datastructures/Posture.hpp>
#include <core_datastructures/Pose.hpp>

namespace convert{
    /**
     * @brief 
     * 
     * @param posture 
     * @return core_datastructures::Posture 
     */
    core_datastructures::Posture to_domain(const common_ros2::msg::Posture& posture);


    /**
     * @brief 
     * 
     * @return common_ros2::msg::Possture 
     */
    common_ros2::msg::Possture to_ros2(const core_datastructures::Posture);
}

