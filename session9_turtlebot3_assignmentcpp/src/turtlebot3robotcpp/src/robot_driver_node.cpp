#include "turtlebot3robotcpp/robot_driver_node.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

RobotDriver::RobotDriver() : Node("robot_driver_node") {
    // Initialize callback groups
    timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Create publisher
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create client for FindClosestWall service
    client = this->create_client<navigation_robot_custom_interfaces::srv::FindClosestWall>("find_closest_wall");

    // Wait for the service to become available
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create subscription
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscription_callback_group;

    subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::QoS(10), 
        std::bind(&RobotDriver::scan_callback, this, std::placeholders::_1),
        subscription_options 
    );

    // Create timer
    timer = this->create_wall_timer(1s, std::bind(&RobotDriver::call_find_closest_wall, this), timer_callback_group);
}

void RobotDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr data) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear = geometry_msgs::msg::Vector3();
    msg.angular = geometry_msgs::msg::Vector3();
    
    float front_obstacle_distance = 0.6f;  // Distance threshold for front obstacle
    float side_obstacle_distance = 0.65f;  // Distance threshold for side obstacle
    
    if (data->ranges[0] < front_obstacle_distance) {
        msg.linear.x = 0.0;
        msg.angular.z = -0.4;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected very close at %.2f meters. Stopping and turning sharply.", data->ranges[0]);
    } else if (data->ranges[35] < side_obstacle_distance) {
        msg.linear.x = 0.0;
        msg.angular.z = -0.2;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected nearby at %.2f meters. Turning slightly.", data->ranges[35]);
    } else {
        msg.linear.x = 0.3;
        msg.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "No immediate obstacle detected. Moving forward with slight speed.");
    }

    publisher->publish(msg);
}

void RobotDriver::call_find_closest_wall() {
    auto request = std::make_shared<navigation_robot_custom_interfaces::srv::FindClosestWall::Request>();
    using ServiceResponseFuture = 
        rclcpp::Client<navigation_robot_custom_interfaces::srv::FindClosestWall>::SharedFuture;

    auto response_received_callback = 
        [this](ServiceResponseFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Closest wall at x: %.2f, y: %.2f, z: %.2f", 
                            response->x, response->y, response->z);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        };

    auto future_result = client->async_send_request(request, response_received_callback);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // Create a single instance of the node
    auto robot_driver = std::make_shared<RobotDriver>();

    // Use a multi-threaded executor if necessary
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_driver);

    // Spin the executor to process the callbacks
    executor.spin();

    // Shutdown ROS 2 and clean up
    rclcpp::shutdown();
    return 0;
}



