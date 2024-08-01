#include "turtlebot3robotcpp/lap_time_server.hpp"

LapTimeServer::LapTimeServer()
: Node("lap_time_server"),
  current_position_(0.0f, 0.0f),
  lap_active_(false)
{
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create action server
    action_server_ = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "measure_lap_time",
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MeasureLapTime>> goal_handle) {
            std::thread([this, goal_handle]() {
                this->execute_callback(goal_handle);
            }).detach();
        },
        [](const std::shared_ptr<GoalHandleMeasureLapTime::Feedback> feedback) {
            // Handle feedback (if needed)
        },
        [](const std::shared_ptr<GoalHandleMeasureLapTime::Result> result) {
            // Handle result (if needed)
        }
    );

    // Create subscription for odometry data
    odometry_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "odometry",
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            this->odometry_callback(msg);
        }
    );
    
    this->get_logger().info("LapTime server started!");
}

void LapTimeServer::odometry_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_position_ = std::make_tuple(msg->pose.pose.position.x, msg->pose.pose.position.y);
    current_time_ = this->now();
}

void LapTimeServer::execute_callback(const GoalHandleMeasureLapTime::SharedPtr goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "LapTime request received. Waiting for robot to get to the starting point");

    if (!wait_for_starting_point()) {
        goal_handle->abort(std::make_shared<MeasureLapTime::Result>());
        return;
    }

    start_time_ = this->now();
    lap_active_ = true;
    auto feedback_msg = std::make_shared<MeasureLapTime::Feedback>();

    // Timer callback to send feedback at regular intervals
    feedback_timer_ = this->create_wall_timer(
        200ms,
        [this, goal_handle, feedback_msg]() {
            if (!lap_active_) {
                return;
            }
            feedback_msg->elapsed_time = (this->now() - start_time_).seconds();
            goal_handle->publish_feedback(feedback_msg);
        }
    );

    if (!wait_for_lap_completion()) {
        goal_handle->abort(std::make_shared<MeasureLapTime::Result>());
        return;
    }

    auto result = std::make_shared<MeasureLapTime::Result>();
    result->total_time = (this->now() - start_time_).seconds();
    lap_active_ = false;
    goal_handle->succeed(result);
}

bool LapTimeServer::wait_for_starting_point()
{
    float tolerance = 0.1f;
    float start_x, start_y;
    std::tie(start_x, start_y) = current_position_;

    while (!is_at_starting_point(start_x, start_y, tolerance)) {
        rclcpp::spin_some(this->get_node_base_interface());
    }

    RCLCPP_INFO(this->get_logger(), "Robot is at the starting point. Starting lap timing.");
    return true;
}

bool LapTimeServer::is_at_starting_point(float start_x, float start_y, float tolerance)
{
    float current_x, current_y;
    std::tie(current_x, current_y) = current_position_;
    float distance = std::sqrt(std::pow(current_x - start_x, 2) + std::pow(current_y - start_y, 2));
    return distance < tolerance;
}

bool LapTimeServer::wait_for_lap_completion()
{
    float start_x, start_y;
    std::tie(start_x, start_y) = current_position_;
    float tolerance = 0.1f;
    int laps_needed = 1;

    RCLCPP_INFO(this->get_logger(), "Waiting for the robot to complete the lap...");
    while (laps_needed > 0) {
        if (is_at_starting_point(start_x, start_y, tolerance)) {
            RCLCPP_INFO(this->get_logger(), "Robot has reached the starting point again. Laps left: %d", laps_needed - 1);
            laps_needed--;
        }
        rclcpp::spin_some(this->get_node_base_interface());
    }

    RCLCPP_INFO(this->get_logger(), "Lap completed.");
    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LapTimeServer>());
    rclcpp::shutdown();
    return 0;
}
