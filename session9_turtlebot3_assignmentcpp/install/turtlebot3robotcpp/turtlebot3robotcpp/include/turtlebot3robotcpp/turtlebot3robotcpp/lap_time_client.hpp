#ifndef TURTLEBOT3ROBOTCPP__LAP_TIME_CLIENT_HPP_
#define TURTLEBOT3ROBOTCPP__LAP_TIME_CLIENT_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "navigation_robot_custom_interfaces/action/measure_lap_time.hpp"

class LapTimeClient : public rclcpp::Node
{
public:
    LapTimeClient();
    void sendGoal();
private:
    
    using MeasureLapTime = navigation_robot_custom_interfaces::action::MeasureLapTime;
    using MeasureLapTimeGoalHandle = rclcpp_action::ClientGoalHandle<MeasureLapTime>;

    void onGoalResponse(const MeasureLapTimeGoalHandle::SharedPtr & goal_handle);
    void onFeedback(
    const MeasureLapTimeGoalHandle::SharedPtr & goal_handle,
    const std::shared_ptr<const MeasureLapTime::Feedback> feedback
);

    void onResult(const MeasureLapTimeGoalHandle::WrappedResult & result);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<MeasureLapTime>::SharedPtr action_client_;
};

#endif  // TURTLEBOT3ROBOTCPP__LAP_TIME_CLIENT_HPP_


