#include "turtlebot3robotcpp/wall_finder_server.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>
#include <thread> // For std::this_thread::sleep_for

using namespace std::chrono_literals;

WallFinder::WallFinder()
    : Node("wall_finder_service_server"), scanning_(false),
      closest_wall_x_(std::numeric_limits<float>::infinity()),
      closest_wall_y_(std::numeric_limits<float>::infinity()),
      closest_wall_z_(0.0f) {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    srv_ = this->create_service<navigation_robot_custom_interfaces::srv::FindClosestWall>(
        "find_closest_wall",
        std::bind(&WallFinder::findWallCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&WallFinder::scanCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WallFinder::odomCallback, this, std::placeholders::_1));
}

WallFinder::~WallFinder() {
    // Clean-up resources if needed
}

void WallFinder::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto start = std::chrono::high_resolution_clock::now();

    scan_data_ = msg->ranges;
    if (!scanning_) return;

    float min_distance = std::numeric_limits<float>::infinity();
    float angle_of_min_distance = 0.0f;

    for (size_t i = 0; i < scan_data_.size(); ++i) {
        if (std::isfinite(scan_data_[i]) && scan_data_[i] < min_distance) {
            min_distance = scan_data_[i];
            angle_of_min_distance = i * msg->angle_increment;
        }
    }

    if (min_distance != std::numeric_limits<float>::infinity()) {
        closest_wall_x_ = min_distance * cos(angle_of_min_distance);
        closest_wall_y_ = min_distance * sin(angle_of_min_distance);
        closest_wall_z_ = 0.0f;  // Assuming 2D environment; adjust if needed
    } else {
        RCLCPP_INFO(this->get_logger(), "No valid wall detected.");
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "scanCallback execution time: %f seconds", elapsed.count());
}

void WallFinder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto start = std::chrono::high_resolution_clock::now();

    auto position = msg->pose.pose.position;
    path_.emplace_back(position.x, position.y);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "odomCallback execution time: %f seconds", elapsed.count());
}

void WallFinder::findWallCallback(
    const std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall::Request> /* request */,
    std::shared_ptr<navigation_robot_custom_interfaces::srv::FindClosestWall::Response> response) {
    
    auto start = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "Incoming FindClosestWall request.");

    scanning_ = true;
    // Set a timer to stop scanning after a certain duration
    auto scan_duration = std::chrono::seconds(5); // Example: 5 seconds
    auto scan_end_time = std::chrono::high_resolution_clock::now() + scan_duration;

    while (scanning_) {
        std::this_thread::sleep_for(100ms); // Avoid busy waiting, sleep for 100ms

        // Check if the scanning should be stopped based on time
        if (std::chrono::high_resolution_clock::now() >= scan_end_time) {
            scanning_ = false;
        }
    }

    bool rotate_right = determineRotationDirection();
    geometry_msgs::msg::Twist msg;
    msg.angular.z = rotate_right ? -0.2 : 0.2;
    publisher_->publish(msg);

    bool done = false;
    while (!done) {
        scanning_ = true;
        auto scan_end_time = std::chrono::high_resolution_clock::now() + scan_duration;

        while (scanning_) {
            std::this_thread::sleep_for(100ms); // Avoid busy waiting, sleep for 100ms

            // Check if the scanning should be stopped based on time
            if (std::chrono::high_resolution_clock::now() >= scan_end_time) {
                scanning_ = false;
            }
        }
        done = hasFoundCorrectDirection();
    }

    msg.angular.z = 0.0;
    publisher_->publish(msg);

    response->x = closest_wall_x_;
    response->y = closest_wall_y_;
    response->z = closest_wall_z_;

    if (closest_wall_x_ != std::numeric_limits<float>::infinity() ||
        closest_wall_y_ != std::numeric_limits<float>::infinity()) {
        RCLCPP_INFO(this->get_logger(), "Wall found at position: (%f, %f, %f)", response->x, response->y, response->z);
    } else {
        RCLCPP_INFO(this->get_logger(), "No wall detected yet.");
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "findWallCallback execution time: %f seconds", elapsed.count());
}

bool WallFinder::determineRotationDirection() {
    auto start = std::chrono::high_resolution_clock::now();

    if (scan_data_.empty()) return false;

    size_t segment_size = scan_data_.size() / 4;
    std::vector<float> segment_distances(4, 0.0f);

    for (size_t i = 0; i < 4; ++i) {
        segment_distances[i] = std::accumulate(scan_data_.begin() + i * segment_size,
                                                scan_data_.begin() + (i + 1) * segment_size, 0.0f) / segment_size;
    }

    size_t min_index = std::min_element(segment_distances.begin(), segment_distances.end()) - segment_distances.begin();
    bool result = min_index >= 2;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "determineRotationDirection execution time: %f seconds", elapsed.count());

    return result;
}

bool WallFinder::hasFoundCorrectDirection() {
    auto start = std::chrono::high_resolution_clock::now();

    if (scan_data_.empty()) return false;

    size_t quarter_size = scan_data_.size() / 4;
    float forward_distance = std::accumulate(scan_data_.begin(), scan_data_.begin() + quarter_size, 0.0f) / quarter_size;

    std::vector<float> surrounding_distances(3, 0.0f);
    for (size_t i = 0; i < 3; ++i) {
        surrounding_distances[i] = std::accumulate(scan_data_.begin() + (i + 1) * quarter_size,
                                                   scan_data_.begin() + (i + 2) * quarter_size, 0.0f) / quarter_size;
    }

    bool result = std::all_of(surrounding_distances.begin(), surrounding_distances.end(), [forward_distance](float distance) {
        return forward_distance >= distance;
    });

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "hasFoundCorrectDirection execution time: %f seconds", elapsed.count());

    return result;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFinder>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}



