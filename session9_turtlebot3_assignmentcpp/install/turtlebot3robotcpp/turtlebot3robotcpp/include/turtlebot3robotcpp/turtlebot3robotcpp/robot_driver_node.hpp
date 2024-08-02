#ifndef TURTLEBOT3ROBOTCPP_ROBOT_DRIVER_NODE_HPP
#define TURTLEBOT3ROBOTCPP_ROBOT_DRIVER_NODE_HPP

#include <memory>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "navigation_robot_custom_interfaces/srv/find_closest_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotDriver : public rclcpp::Node {
public:
    RobotDriver();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr data);
    void call_find_closest_wall();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
    rclcpp::Client<navigation_robot_custom_interfaces::srv::FindClosestWall>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer;
    
    // Callback groups
    rclcpp::CallbackGroup::SharedPtr timer_callback_group;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group;
};

#endif


