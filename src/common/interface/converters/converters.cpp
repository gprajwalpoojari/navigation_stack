#include <converters.hpp>
#include <iostream>

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

    sensor_datastructures::IMUData to_domain(const sensor_msgs::msg::Imu& imu_data){
        sensor_datastructures::IMUData new_imu_data;
        new_imu_data.angular_velocity<<imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z;

        new_imu_data.linear_acceleration<<imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z;
        
        new_imu_data.orientation.x() = imu_data.orientation.x;
        new_imu_data.orientation.y() = imu_data.orientation.y;
        new_imu_data.orientation.z() = imu_data.orientation.z;
        new_imu_data.orientation.w() = imu_data.orientation.w;
        
        std::cout << new_imu_data.angular_velocity << std::endl;

        // These will need reshaping to (3, 3)
        // new_imu_data.angular_velocity_covariance = imu_data->angular_velocity_covariance;
        // new_imu_data.linear_acceleration_covariance = imu_data->linear_acceleration_covariance;
        // new_imu_data.orientation_covariance = imu_data->orientation_covariance;
        return new_imu_data;
    }

    sensor_msgs::msg::Imu::SharedPtr to_ros2(const sensor_datastructures::IMUData& imu_data){
        sensor_msgs::msg::Imu::SharedPtr new_imu_data;
        
        return new_imu_data;
    }

    sensor_datastructures::OdomData to_domain(const nav_msgs::msg::Odometry& odom_data){
        sensor_datastructures::OdomData new_odom_data;

        return new_odom_data;
    }

    nav_msgs::msg::Odometry::SharedPtr to_ros2(const sensor_datastructures::OdomData& odom_data){
        nav_msgs::msg::Odometry::SharedPtr new_odom_data;

        return new_odom_data;
    }
}