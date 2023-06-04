#include <trajectory_publisher.hpp>
#include <converters/converters.hpp>
#include <spline_generation/cubic_spline_generator.hpp>
#include <common_ros2/msg/spline.hpp>
#include <trajectory_generation/lattice_trajectory_generator.hpp>
#include <core_datastructures/dynamic_posture.hpp>
#include <common_ros2/msg/trajectory.hpp>


TrajectoryPublisher::TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal)
                                        : Node("trajectory_publisher"), start_(start), goal_(goal) {
  trajectory_generator_ = std::make_shared<trajectory_generation::trajectory_generation::LatticeTrajectoryGenerator>(start, goal);                        
  publisher_ = this->create_publisher<common_ros2::msg::Trajectory>("/trajectory", 10);      
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));

}


void TrajectoryPublisher::timer_callback() {
  if (!callback_executed) {
    auto trajectory_ros = common_ros2::msg::Trajectory();

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
    RCLCPP_INFO(this->get_logger(), "Published");
      callback_executed = true;
  }
      

}


RvizPublisher::RvizPublisher(rclcpp::Node* node,
                  std::string frame_id,
                  float point_scale,
                  float line_scale)
{
  rclcpp::Clock clock;

  points.header.frame_id = line_strip.header.frame_id = frame_id;

  points.header.stamp = line_strip.header.stamp = clock.now();

  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  
  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::msg::Marker::POINTS;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

  points.scale.x =  points.scale.y = point_scale;

  line_strip.scale.x = line_strip.scale.y = line_scale;

  points.color.r = 1.0f;
  points.color.a = 1.0;

  line_strip.color.g = 1.0f;
  line_strip.color.a = 0.5;

  visualize_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker",10);

}

void RvizPublisher::publish()
{
  visualize_->publish(line_strip);
  visualize_->publish(points);
  line_strip.points.clear();
  points.points.clear();
}


void RvizPublisher::add_point(const geometry_msgs::msg::Point& p)
{
  points.points.push_back(p);
}


void RvizPublisher::add_line(const geometry_msgs::msg::Point& p){
  line_strip.points.push_back(p);
}
