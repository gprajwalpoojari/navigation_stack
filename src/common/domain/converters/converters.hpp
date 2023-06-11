#ifndef COMMON_DOMAIN__CONVERTERS__CONVERTERS_HPP
#define COMMON_DOMAIN__CONVERTERS__CONVERTERS_HPP

#include <euler_axis.hpp>
#include <eigen3/Eigen/Dense>


namespace common_domain::converters{
    /**
     * @brief Quaternion to euler angles based on yaw-pitch-roll convention
     * 
     * @param q 
     * @return core_datastructures::EulerAngle 
     */
    core_datastructures::EulerAngle quat_to_eul(const Eigen::Quaterniond& q);

}

#endif // COMMON_DOMAIN__CONVERTERS__CONVERTERS_HPP