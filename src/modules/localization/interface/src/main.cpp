#include <localization_publisher.hpp>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<EKFPublisher>();
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(std::make_shared<EKFPublisher>());
  rclcpp::shutdown();
  return 0;
}