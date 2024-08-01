#ifndef LAP_TIME_SERVER_HPP_
#define LAP_TIME_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <navigation_robot_custom_interfaces/action/measure_lap_time.hpp>
#include <memory>
#include <cmath>

class LapTimeServer : public rclcpp::Node
{
public:
    using MeasureLapTime = navigation_robot_custom_interfaces::action::MeasureLapTime;
    using GoalHandleMeasureLapTime = rclcpp_action::Server<MeasureLapTime>::GoalHandle;

    explicit LapTimeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odometry_subscription_;
    double start_time_;
    bool lap_active_;
    double current_time_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    struct Position
    {
        double x;
        double y;
    } current_position_;

    void odometry_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle);
    bool wait_for_starting_point();
    bool is_at_starting_point(double start_x, double start_y, double tolerance = 0.1);
    bool wait_for_lap_completion();
};

#endif // LAP_TIME_SERVER_HPP_
