#include <trajectory_publisher.hpp>
#include <converters/converters.hpp>
#include <spline_generation/cubic_spline_generator.hpp>
#include <common_ros2/msg/spline.hpp>

TrajectoryPublisher::TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal)
                                        : Node("trajectory_publisher"), start_(start), goal_(goal) {
      
      
      auto spline_generator = std::make_shared<trajectory_generation::spline_generation::CubicSplineGenerator>();
      auto road_center = spline_generator->get_spline(start_, goal_);
      graph_generator_ = std::make_shared<trajectory_generation::graph_generation::GraphGenerator>(spline_generator, road_center);                        
      publisher_ = this->create_publisher<common_ros2::msg::Splines>("/trajectory", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));
    }


void TrajectoryPublisher::timer_callback() {
  if (!callback_executed) {
    auto splines_ros = common_ros2::msg::Splines();
    graph_generator_->search_graph();
    auto splines = graph_generator_->get_spline_lattice();
    for (auto spline : splines) {
      auto spline_ros = common_ros2::msg::Spline();
      for(unsigned int i=0; i<spline.size(); i++){
        spline_ros.spline_points.push_back(converters::to_ros2(spline[i]));
      }
      splines_ros.splines.push_back(spline_ros);
    }
      publisher_->publish(splines_ros);
      RCLCPP_INFO(this->get_logger(), "Published");
      callback_executed = true;
  }
      

}
