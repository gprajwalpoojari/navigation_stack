#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <common_ros2/msg/posture.hpp>
#include <common_ros2/msg/spline.hpp>

#include <spline_generation/cubic_spline_generator.hpp>
#include <core_datastructures/Posture.hpp>
#include <iostream>



class TrajectoryPublisher : public rclcpp::Node
{
  public:
    TrajectoryPublisher(core_datastructures::Posture& start, core_datastructures::Posture& goal )
    : Node("trajectory_publisher"), count_(0), start_(start), goal_(goal)
    {
      publisher_ = this->create_publisher<common_ros2::msg::Spline>("/trajectory", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryPublisher::timer_callback, this));

    }


  private:
    void timer_callback()
    {
             
      auto message = common_ros2::msg::Spline();
     
      trajectory_generation::CubicSplineGenerator spl(start_,goal_);
      std::vector<core_datastructures::Posture> spline = spl.get_spline();
      for(unsigned int i=0; i<spline.size(); i++){
        auto temp = common_ros2::msg::Posture();
        temp.x = spline[i].x;
        temp.y = spline[i].y;
        message.spline_points.push_back(temp);
      }
          

      RCLCPP_INFO(this->get_logger(), "Published");
      publisher_->publish(message);

    }

    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<common_ros2::msg::Spline>::SharedPtr publisher_;
    size_t count_;
    core_datastructures::Posture start_;
    core_datastructures::Posture goal_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  core_datastructures::Posture start{0,0,0,0}, goal{3,8,0,0};
  if(argc == 9){
    start.x = atof(argv[1]);
    start.y = atof(argv[2]);
    start.theta = atof(argv[3]);
    start.kappa = atof(argv[4]);
    goal.x = atof(argv[5]);
    goal.y = atof(argv[6]);
    goal.theta = atof(argv[7]);
    goal.kappa = atof(argv[8]);
  }
  std::string values = "\nStart: x:" + std::to_string(start.x) + " y:" + std::to_string(start.y) + " theta:" + std::to_string(start.theta) + " kappa:" + std::to_string(start.kappa) +
                       "\nGoal:  x:" + std::to_string(goal.x) + " y:" + std::to_string(goal.y) + " theta:" + std::to_string(goal.theta) + " kappa:" + std::to_string(goal.kappa);
  
  RCLCPP_INFO(rclcpp::get_logger("Spline Requested"),"%s", values.c_str());
  rclcpp::spin(std::make_shared<TrajectoryPublisher>(start,goal));
  rclcpp::shutdown();
  return 0;
}