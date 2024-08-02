#include "turtlebot3robotcpp/lap_time_server.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

using MeasureLapTime = navigation_robot_custom_interfaces::action::MeasureLapTime;
using GoalHandleMeasureLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

LapTimeServer::LapTimeServer() : Node("lap_time_server") {
    using namespace std::placeholders;

    // Create callback groups
    service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize action server
    this->action_server = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "lap_time",
        std::bind(&LapTimeServer::handle_goal, this, _1, _2),
        std::bind(&LapTimeServer::handle_cancel, this, _1),
        std::bind(&LapTimeServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        service_cb_group
    );

    // Initialize odometry subscriber
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscription_cb_group;
    subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&LapTimeServer::odometry_callback, this, _1),
        subscription_options
    );

    // Initialize variables
    x = y = start_x = start_y = 2.0;
    feedback_msg = std::make_shared<MeasureLapTime::Feedback>();

    RCLCPP_INFO(this->get_logger(), "MeasureLapTime server started. Ready for requests.");
}

void LapTimeServer::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
}

rclcpp_action::GoalResponse LapTimeServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MeasureLapTime::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request.");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LapTimeServer::handle_cancel(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeServer::handle_accepted(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&LapTimeServer::execute, this, _1), goal_handle}.detach();
}

void LapTimeServer::execute(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Waiting for robot to reach starting point...");

    // Wait for the robot to reach the starting point
    while (rclcpp::ok() && !goal_handle->is_canceling() && !is_near_start_position()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled before reaching the starting point.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Robot reached starting point.");
    start_x = x;
    start_y = y;
    start_time = this->now(); 

    // Create timer to publish feedback
    timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this, goal_handle]() { this->publish_elapsed_time(goal_handle); }, 
        timer_cb_group
    );

    bool lap_completed = false;
    
    

    // Main execution loop
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
        // Check if the robot is near the start position and if the lap has not been completed yet
        if (is_near_start_position() && !lap_completed && (this->now() - start_time) > rclcpp::Duration(1, 0)) {
            // Ensure the robot is actually at the start position for a certain period before concluding
            auto post_check_duration = rclcpp::Duration(5, 0); // 5 seconds for additional verification
            auto check_start_time = this->now();
            while (rclcpp::ok() && !goal_handle->is_canceling() && (this->now() - check_start_time) < post_check_duration) {
                if (is_near_start_position()) {
                    auto result = std::make_shared<MeasureLapTime::Result>();
                    result->total_time = (this->now() - start_time).seconds(); 
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded. Total lap time: %f", result->total_time);
                    lap_completed = true;
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Handle cancellation
    if (goal_handle->is_canceling()) {
        auto result = std::make_shared<MeasureLapTime::Result>();
        result->total_time = (this->now() - start_time).seconds(); 
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    }
}

void LapTimeServer::publish_elapsed_time(const std::shared_ptr<GoalHandleMeasureLapTime> goal_handle) {
    feedback_msg->elapsed_time = (this->now() - start_time).seconds(); 
    goal_handle->publish_feedback(feedback_msg);
}

bool LapTimeServer::is_near_start_position() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    double distance = std::sqrt(std::pow(x - start_x, 2) + std::pow(y - start_y, 2));
    return distance < 0.5; 
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<LapTimeServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


