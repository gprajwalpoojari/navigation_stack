#include <trajectory_publisher.hpp>
#include <converters/converters.hpp>
#include <trajectory_generation/lattice_trajectory_generator.hpp>
#include <core_datastructures/dynamic_posture.hpp>


TrajectoryPublisher::TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal)
                                        : Node("trajectory_publisher"), start_(start), goal_(goal)
                                         {
  trajectory_generator_ = std::make_shared<trajectory_generation::trajectory_generation::LatticeTrajectoryGenerator>(start, goal);                        
  publisher_ = this->create_publisher<common_ros2::msg::Trajectory>("/trajectory", 10);   
  visualizer_ = std::make_shared<RvizPublisher>(this);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));

}


void TrajectoryPublisher::timer_callback() {
  if (!callback_executed) {
    common_ros2::msg::Trajectory trajectory_ros;
    double velocity = 0;
    double time = 0;
    double acceleration = 1;
    core_datastructures::DynamicPosture start_dyn_posture{start_.x, start_.y, start_.theta, start_.kappa, velocity, time};
    std::vector<core_datastructures::DynamicPosture> trajectory_core = trajectory_generator_->
                                                                      generate_trajectory(start_dyn_posture, goal_, acceleration);
    for (std::size_t i = 0; i < trajectory_core.size(); i++) {
      trajectory_ros.trajectory_points.push_back(converters::to_ros2(trajectory_core[i]));
    }
    publisher_->publish(trajectory_ros);
    visualizer_->publish(trajectory_ros);
    RCLCPP_INFO(this->get_logger(), "Published");
      callback_executed = true;
  }
}