#ifndef CONVERTERS__CONVERTERS_HPP
#define CONVERTERS__CONVERTERS_HPP

#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/dynamic_posture.hpp>
#include <core_datastructures/posture.hpp>
#include <core_datastructures/dynamic_posture.hpp>
#include <sensor_datastructures/imu.hpp>
#include <sensor_datastructures/odom.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

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

    /**
     * @brief Convert ros2 IMU Data to domain
     * 
     * @param imu_data
     * @return sensor_datastructures::IMUData
     */
    sensor_datastructures::IMUData to_domain(const sensor_msgs::msg::Imu& imu_data);

    /**
     * @brief Convert ros2 Odom Data to domain
     * 
     * @param odom_data
     * @return sensor_datastructures::OdomData
     */
    sensor_datastructures::OdomData to_domain(const nav_msgs::msg::Odometry& odom_data);

}

#endif // CONVERTERS__CONVERTERS_HPP