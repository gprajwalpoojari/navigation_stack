#ifndef CONVERTERS__CONVERTERS_HPP
#define CONVERTERS__CONVERTERS_HPP

#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/dynamic_posture.hpp>
#include <core_datastructures/posture.hpp>
#include <core_datastructures/dynamic_posture.hpp>

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


    /**
     * @brief Convert ros2 Posture to domain
     * 
     * @param dyn_posture 
     * @return core_datastructures::DynamicPosture 
     */
    core_datastructures::DynamicPosture to_domain(const common_ros2::msg::DynamicPosture& dyn_posture);


    /**
     * @brief Convert domain Posture to ros2
     * 
     * @param posture
     * @return common_ros2::msg::DynamicPosture 
     */
    common_ros2::msg::DynamicPosture to_ros2(const core_datastructures::DynamicPosture& dyn_posture);

}

#endif // CONVERTERS__CONVERTERS_HPP