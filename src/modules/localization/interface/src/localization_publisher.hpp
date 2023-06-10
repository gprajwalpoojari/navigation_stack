#ifndef LOCALIZATION__EKF_PUBLISHER
#define LOCALIZATION__EKF_PUBLISHER


#include <extended_kalman_filter/measurement_package.hpp>
#include <extended_kalman_filter/track_filter.hpp>
#include <vector>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"



/**
 * @brief EKF Publisher Node that performs ekf and publishes over a topic.
 * 
 */
class EKFPublisher : public rclcpp::Node
{

private:

    nav_msgs::msg::Path msg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
    std::vector<localization::extended_kalman_filter::MeasurementPackage> measurements;  

    /**
     * @brief This is a timer callback for the /ekf_states topic publisher
     * 
     */
    void timer_callback();
  
  public:
    /**
     * @brief Construct a new EKFPublisher object
     * 
     * @param measurements 
     */
    EKFPublisher();
    
    /**
     * @brief Perform EKF and sensor data and load it in ROS2 navigation Path message
     * 
     * @param measurements 
     * @return nav_msgs::msg::Path 
     */
    nav_msgs::msg::Path load_msg(const std::vector<localization::extended_kalman_filter::MeasurementPackage>& measurements) const;


    /**
     * @brief This function loads the EKF Data in a vector
     * 
     * @return std::vector<localization::extended_kalman_filter::MeasurementPackage> 
     */
    std::vector<localization::extended_kalman_filter::MeasurementPackage> read_data() const;


};

#endif // LOCALIZATION__EKF_PUBLISHER

