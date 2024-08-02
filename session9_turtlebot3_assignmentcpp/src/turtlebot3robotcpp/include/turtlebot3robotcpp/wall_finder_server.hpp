#ifndef WALL_FINDER_SERVER_HPP
#define WALL_FINDER_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <navigation_robot_custom_interfaces/srv/find_closest_wall.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

class WallFinder : public rclcpp::Node {
public:
    WallFinder();
    ~WallFinder();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void findWallCallback(
        const std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall::Request> /* request */,
        std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall::Response> response);

    bool determineRotationDirection();
    bool hasFoundCorrectDirection();
    bool scanning_;
    float closest_wall_x_;
    float closest_wall_y_;
    float closest_wall_z_;
    std::vector<float> scan_data_;
    std::vector<std::pair<float, float>> path_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<navigation_robot_custom_interfaces::srv::FindClosestWall>::SharedPtr srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

#endif // WALL_FINDER_SERVER_HPP


