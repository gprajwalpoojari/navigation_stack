#include <localization_publisher.hpp>


int main(int argc, char * argv[])
{
  auto measurements = read_data();
//   std::vector<localization::extended_kalman_filter::MeasurementPackage> measurements;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFPublisher>(measurements));
  rclcpp::shutdown();
  return 0;
}