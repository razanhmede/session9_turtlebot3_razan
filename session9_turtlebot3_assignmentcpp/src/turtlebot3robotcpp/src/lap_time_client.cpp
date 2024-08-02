#include "turtlebot3robotcpp/lap_time_client.hpp"
#include <chrono>
#include <iomanip>  // For formatting floating-point numbers

using MeasureLapTime = navigation_robot_custom_interfaces::action::MeasureLapTime;
using MeasureLapTimeGoalHandle = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

LapTimeClient::LapTimeClient() : Node("lap_time_client") {
    action_client_ = rclcpp_action::create_client<MeasureLapTime>(this, "lap_time");
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LapTimeClient::sendGoal, this)
    );
}

void LapTimeClient::sendGoal() {
    if (!action_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    auto goal_message = MeasureLapTime::Goal();
    auto goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(
        &LapTimeClient::onGoalResponse,
        this,
        std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &LapTimeClient::onFeedback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &LapTimeClient::onResult,
        this,
        std::placeholders::_1
    );

    action_client_->async_send_goal(goal_message, goal_options);
}

void LapTimeClient::onGoalResponse(const MeasureLapTimeGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected.");
        rclcpp::shutdown();
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted. Awaiting execution.");
    }
}

void LapTimeClient::onFeedback(
    const MeasureLapTimeGoalHandle::SharedPtr &,
    const std::shared_ptr<const MeasureLapTime::Feedback> feedback
) {
    std::stringstream feedback_message;
    feedback_message << "Elapsed time: " << std::fixed << std::setprecision(2) 
                     << feedback->elapsed_time << " seconds.";
    RCLCPP_INFO(this->get_logger(), feedback_message.str().c_str());
}

void LapTimeClient::onResult(const MeasureLapTimeGoalHandle::WrappedResult & result) {
    std::stringstream result_message;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_message << "Lap completed in: " << std::fixed << std::setprecision(2) 
                           << result.result->total_time << " seconds.";
            RCLCPP_INFO(this->get_logger(), result_message.str().c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Action was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Action was canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
    }
    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<LapTimeClient>();
    client_node->sendGoal();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}

