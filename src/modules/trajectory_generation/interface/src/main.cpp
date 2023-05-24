#include <trajectory_publisher.hpp>
#include <string>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  core_datastructures::Posture start{0,0,0,0}, goal{10,1,0,0};
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
