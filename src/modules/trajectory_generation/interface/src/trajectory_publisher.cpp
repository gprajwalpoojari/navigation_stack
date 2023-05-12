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
using namespace std::chrono_literals;
using trajectory_generation::CubicSplineGenerator;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TrajectoryPublisher : public rclcpp::Node
{
  public:
    TrajectoryPublisher()
    : Node("trajectory_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<common_ros2::msg::Spline>("/trajectory", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
             
      auto message = common_ros2::msg::Spline();
     
      core_datastructures::Posture start{0,0,0,0}, goal{4,0,0,0};

      
      trajectory_generation::CubicSplineGenerator spl(start,goal);
      std::vector<core_datastructures::Posture> spline = spl.get_spline();
      std::cout<<spline.size()<<std::endl;
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
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}