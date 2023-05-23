#ifndef TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <common_ros2/msg/spline.hpp>
#include <core_datastructures/Posture.hpp>

/** 
 * @brief Trajectory Publisher class for publishing trajectories
 * 
*/
class TrajectoryPublisher : public rclcpp::Node {
  public:
    /** @brief Constructor for the class
     * 
     * @param[in] start
     * @param[in] goal 
     *  
    */
    TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal);
    
  private:

    /**
     * @brief callback that specifies the logic for each iteration
    */
    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<common_ros2::msg::Spline>::SharedPtr publisher_;
    size_t count_;
    core_datastructures::Posture start_;
    core_datastructures::Posture goal_;
};



#endif /*TRAJECTORY_PUBLISHER_HPP*/