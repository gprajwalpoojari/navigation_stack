#include <trajectory_publisher.hpp>
#include <converters/converters.hpp>
#include <spline_generation/cubic_spline_generator.hpp>
#include <common_ros2/msg/spline.hpp>

TrajectoryPublisher::TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal )
    : Node("trajectory_publisher"), count_(0), start_(start), goal_(goal) {
      publisher_ = this->create_publisher<common_ros2::msg::Spline>("/trajectory", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));
    }


void TrajectoryPublisher::timer_callback() {
  auto message = common_ros2::msg::Spline();
  trajectory_generation::CubicSplineGenerator spl(start_,goal_);
  std::vector<core_datastructures::Posture> spline = spl.get_spline();
  for(unsigned int i=0; i<spline.size(); i++){
    message.spline_points.push_back(converters::to_ros2(spline[i]));
  }
      
  RCLCPP_INFO(this->get_logger(), "%u", spline.size());
  publisher_->publish(message);

}
