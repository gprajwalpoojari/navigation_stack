#include <localization_publisher.hpp>
#include <iostream>
#include <fstream>
using std::placeholders::_1;


EKFPublisher::EKFPublisher(): Node("ekf_publisher"), count_(0)
{
  publisher_ = this->create_publisher<nav_msgs::msg::Path>("/ekf_states", 1000);
  imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&EKFPublisher::imu_callback, this, _1));
  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1000, std::bind(&EKFPublisher::odom_callback, this, _1));
  // measurement = get_measurement();
  // measurements = read_data();
  // msg = load_msg(measurements);
  // timer_ = this->create_wall_timer(
  // 500ms, std::bind(&EKFPublisher::timer_callback, this));

}

void EKFPublisher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
  sensor_datastructures::IMUData imu_data = converters::to_domain(*msg);
  RCLCPP_INFO(this->get_logger(),"Recieving IMU DATA");
  // tracker.measurement_update_IMU(imu_data,0.00);

}

void EKFPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  sensor_datastructures::OdomData odom_data = converters::to_domain(*msg);
  RCLCPP_INFO(this->get_logger(),"Recieving ODOM DATA");
  // tracker.measurement_update_Odom(odom_data,0.00);
}

// nav_msgs::msg::Path EKFPublisher::load_msg(const std::vector<localization::extended_kalman_filter::MeasurementPackage>& measurements) const
// {
//   nav_msgs::msg::Path message;
//   geometry_msgs::msg::PoseStamped pose1;
//   size_t N = measurements.size();
//   localization::extended_kalman_filter::Tracker t;
//   for(size_t k = 0;k<N;++k){
//       t.measurement_update(measurements[k]);
//       pose1.pose.position.x = t.states(0);
//       pose1.pose.position.y = t.states(1);
//       message.poses.push_back(pose1);
//   }
//   return message;
// }

void EKFPublisher::timer_callback(){
  msg.header.frame_id = "Path";
  msg.header.stamp = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.header.frame_id.c_str());
  publisher_->publish(msg);
}

// std::vector<localization::extended_kalman_filter::MeasurementPackage> EKFPublisher::read_data() const{
//   std::vector<localization::extended_kalman_filter::MeasurementPackage> measure_pack_list;
//   std::string filename = "/home/pinak/Tensor Robotics/navigation_stack/src/modules/localization/interface/dev_tools/obj_pose-laser-radar-synthetic-input.txt";
//   std::ifstream ifs;
//   ifs.open(filename.c_str(),std::ifstream::in);

//   if (!ifs.is_open()) {
//   std::cout << "Cannot open input file: " <<filename<< std::endl;
//   }

//   std::string line;
//   while(getline(ifs,line)){
//       localization::extended_kalman_filter::MeasurementPackage packet;
//       std::istringstream iss(line);
//       std::string sensor_type;
//       iss >> sensor_type;
//       int64_t timestamp;

//       if(sensor_type.compare("L") == 0){
//           packet.sensor_type = localization::extended_kalman_filter::SensorType::LASER;
//           packet.raw_measurements_ = Eigen::VectorXd(2);
//           float x,y;
//           iss>>x;
//           iss>>y;
//           packet.raw_measurements_<<x,y;
//           iss>>timestamp;
//           packet.timestamp_ = timestamp;
//           measure_pack_list.push_back(packet);
//       }
//       else if(sensor_type.compare("R") == 0){
//           packet.sensor_type = localization::extended_kalman_filter::SensorType::RADAR;
//           packet.raw_measurements_ = Eigen::VectorXd(3);
//           float rho,phi,rho_dot;
//           iss>>rho;
//           iss>>phi;
//           iss>>rho_dot;
//           packet.raw_measurements_<<rho,phi,rho_dot;
//           iss>>timestamp;
//           packet.timestamp_ = timestamp;
//           measure_pack_list.push_back(packet);

//       }
//   }
//   return measure_pack_list;
// }

  



