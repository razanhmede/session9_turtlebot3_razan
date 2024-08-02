#ifndef TURTLEBOT3ROBOTCPP__LAP_TIME_SERVER_HPP_
#define TURTLEBOT3ROBOTCPP__LAP_TIME_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigation_robot_custom_interfaces/action/measure_lap_time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>
#include <chrono>

class LapTimeServer : public rclcpp::Node
{
public:
    LapTimeServer();

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const navigation_robot_custom_interfaces::action::MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_robot_custom_interfaces::action::MeasureLapTime>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_robot_custom_interfaces::action::MeasureLapTime>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_robot_custom_interfaces::action::MeasureLapTime>> goal_handle);
    void publish_elapsed_time(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_robot_custom_interfaces::action::MeasureLapTime>> goal_handle);
    bool is_near_start_position();

    rclcpp_action::Server<navigation_robot_custom_interfaces::action::MeasureLapTime>::SharedPtr action_server;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;
    rclcpp::CallbackGroup::SharedPtr service_cb_group;
    rclcpp::CallbackGroup::SharedPtr subscription_cb_group;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group;
    std::mutex odom_mutex_;
    double x, y, start_x, start_y;
    rclcpp::Time start_time;
    std::shared_ptr<navigation_robot_custom_interfaces::action::MeasureLapTime::Feedback> feedback_msg;
    rclcpp::TimerBase::SharedPtr timer;
};
#endif  // TURTLEBOT3ROBOTCPP__LAP_TIME_SERVER_HPP_



