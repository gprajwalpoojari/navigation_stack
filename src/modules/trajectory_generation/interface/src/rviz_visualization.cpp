#include <rviz_visualization.hpp>

RvizPublisher::RvizPublisher(rclcpp::Node* node,
                  std::string frame_id,
                  float point_scale,
                  float line_scale)
{

  points.header.frame_id = line_strip.header.frame_id = frame_id;

  points.header.stamp = line_strip.header.stamp = node->get_clock()->now();

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

void RvizPublisher::publish(const common_ros2::msg::Trajectory& trajectory)
{ 
  geometry_msgs::msg::Point start_point;
  start_point.x = trajectory.trajectory_points[0].x;
  start_point.y = trajectory.trajectory_points[0].y;
  start_point.z = trajectory.trajectory_points[0].v;
  points.points.push_back(start_point);

  for (const auto& trajectory_point :trajectory.trajectory_points) {
    geometry_msgs::msg::Point point;
    point.x = trajectory_point.x;
    point.y = trajectory_point.y;
    point.z = 0;
    line_strip.points.push_back(point);
  }
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

