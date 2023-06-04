#include <trajectory_publisher.hpp>
#include <converters/converters.hpp>
#include <spline_generation/cubic_spline_generator.hpp>
#include <common_ros2/msg/spline.hpp>
#include <trajectory_generation/lattice_trajectory_generator.hpp>
#include <core_datastructures/dynamic_posture.hpp>

TrajectoryPublisher::TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal)
                                        : Node("trajectory_publisher"), start_(start), goal_(goal),
                                          rviz(RvizPublisher(this, "map", 0.05, 0.01))
                                          // , my_point_color, my_line_color));
 {
      
      
      auto spline_generator = std::make_shared<trajectory_generation::spline_generation::CubicSplineGenerator>(start, goal);
      // auto trajectory_generation::trajectory_generation::LatticeTrajectoryGenerator temp(start, goal);
      // core_datastructures::DynamicPosture
      // auto var = temp.generate_trajectory()
      auto road_center = spline_generator->get_spline(start_, goal_);
      graph_generator_ = std::make_shared<trajectory_generation::graph_generation::GraphGenerator>(spline_generator, road_center);                        
      publisher_ = this->create_publisher<common_ros2::msg::Splines>("/trajectory", 10);
      // visualize_path_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_path",0);
      
      
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));
      // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this, &rviz]() {
      //   timer_callback(rviz);
      // });
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
        
        geometry_msgs::msg::Point p;
        p.x = spline_ros.spline_points[i].x;
        p.y = spline_ros.spline_points[i].y;
        p.z = 0;
        if(i==0 || i==spline.size()-1){
          rviz.add_point(p);
        }
        rviz.add_line(p);
      }
      splines_ros.splines.push_back(spline_ros);
    }
      publisher_->publish(splines_ros);
      rviz.publish();
      RCLCPP_INFO(this->get_logger(), "Published");
      callback_executed = false;
  }
      

}


RvizPublisher::RvizPublisher(rclcpp::Node* node,
                  std::string frame_id,
                  float point_scale,
                  float line_scale)
                  // ,
                  // float* point_color,
                  // float* line_color)
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
  // float point_color[3] = {1.0f, 0.0f, 0.0f};
  // float line_color[3] = {0.0f, 1.0f, 0.0f};


  line_strip.scale.x = line_strip.scale.y = line_scale;

  points.color.r = 1.0f;
  points.color.a = 1.0;

  line_strip.color.g = 1.0f;
  line_strip.color.a = 0.5;

  visualize_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker",10);
  // executed = false;
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RvizPublisher::timer_callback, this));

}

void RvizPublisher::publish()
{
  std::cout << "Pusblishing to Rviz... : " << line_strip.points.size() << std::endl;
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

// void RvizPublisher::timer_callback(){
//   while(!executed){
//     if(visualize_.use_count()>0){
//       publish();
//       // executed = true;
//     }
//   }
//   // executed = false;
// }