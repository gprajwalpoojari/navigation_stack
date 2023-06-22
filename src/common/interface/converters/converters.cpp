#include <converters.hpp>

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

        double orientation_cov_array[9];
        double angular_velocity_cov_array[9];
        double linear_acceleration_cov_array[9];
        for(int i=0; i<9; i++){
            orientation_cov_array[i] = imu_data.orientation_covariance[i];
            angular_velocity_cov_array[i] = imu_data.angular_velocity_covariance[i];
            linear_acceleration_cov_array[i] = imu_data.linear_acceleration_covariance[i];
        }

        new_imu_data.orientation_covariance = Eigen::Map< Eigen::Matrix3d>(orientation_cov_array);
        new_imu_data.angular_velocity_covariance = Eigen::Map< Eigen::Matrix3d>(angular_velocity_cov_array);
        new_imu_data.linear_acceleration_covariance = Eigen::Map< Eigen::Matrix3d>(linear_acceleration_cov_array);

        return new_imu_data;
    }

    sensor_datastructures::OdomData to_domain(const nav_msgs::msg::Odometry& odom_data){
        sensor_datastructures::OdomData new_odom_data;
        new_odom_data.position<<odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z;

        new_odom_data.orientation.x() = odom_data.pose.pose.orientation.x;
        new_odom_data.orientation.y() = odom_data.pose.pose.orientation.y;
        new_odom_data.orientation.z() = odom_data.pose.pose.orientation.z;
        new_odom_data.orientation.w() = odom_data.pose.pose.orientation.w;

        new_odom_data.linear_velocity<<odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y, odom_data.twist.twist.linear.z;
        new_odom_data.angular_velocity<<odom_data.twist.twist.angular.x, odom_data.twist.twist.angular.y, odom_data.twist.twist.angular.z;

        double pose_cov_array[36];
        double twist_cov_array[36];
        for(int i=0; i<36; i++){
            pose_cov_array[i] = odom_data.pose.covariance[i];
            twist_cov_array[i] = odom_data.twist.covariance[i];
        }
        new_odom_data.pose_covariance = Eigen::Map< Eigen::Matrix<double, 6, 6>>(pose_cov_array);
        new_odom_data.twist_covariance = Eigen::Map< Eigen::Matrix<double, 6, 6>>(twist_cov_array);

        return new_odom_data;
    }

    Eigen::Vector3d to_domain(const geometry_msgs::msg::Twist& cmd_vel_msg){
        Eigen::Vector3d control_input;
        control_input << cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z;
        return control_input;
    }
}