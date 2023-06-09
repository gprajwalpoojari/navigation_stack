#ifndef RVIZ_VISUALIZE_HPP
#define RVIZ_VISUALIZE_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <string>
#include <common_ros2/msg/trajectory.hpp>

/**
 * @brief Class to publish trajectory in Rviz
 * 
 */
class RvizPublisher 
{
public:
    /**
     * @brief Construct a new Rviz Publisher object
     * 
     * @param node pointer to the current node
     * @param frame_id frame_id name to be selected in Rviz
     * @param point_scale visualization scale for points
     * @param line_scale visualization scale for line
     */
    RvizPublisher(rclcpp::Node* node,
                    std::string frame_id="map",
                    float point_scale=0.1,
                    float line_scale=0.05);

    /**
     * @brief publish to the Marker topic
     * 
     */
    void publish(const common_ros2::msg::Trajectory& trajectory);

    /**
     * @brief adds the passed point to Marker.points to be published
     * 
     * @param p 
     */
    void add_point(const geometry_msgs::msg::Point& p);

    /**
     * @brief adds the passed point to Marker.line to be published
     * 
     * @param p 
     */
    void add_line(const geometry_msgs::msg::Point& p);

    private:
    /**
     * @brief Publisher for visualization_msgs::msg::Marker topic
     * 
     */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_;

    /**
     * @brief points conatiner to store visulaizing points
     * 
     */
    visualization_msgs::msg::Marker points;

    /**
     * @brief line containe to store points for visualizing line
     * 
     */
    visualization_msgs::msg::Marker line_strip;
};


#endif /*RVIZ_VISUALIZE_HPP*/
