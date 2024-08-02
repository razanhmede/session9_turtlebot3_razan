cmake_minimum_required(VERSION 3.8)
project(turtlebot3robotcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navigation_robot_custom_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(include)

# Add executables
add_executable(robot_driver_node src/robot_driver_node.cpp)
ament_target_dependencies(robot_driver_node rclcpp std_msgs sensor_msgs geometry_msgs navigation_robot_custom_interfaces)

add_executable(wall_finder_server src/wall_finder_server.cpp)
ament_target_dependencies(wall_finder_server rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs navigation_robot_custom_interfaces)
add_executable(lap_time_server src/lap_time_server.cpp)
ament_target_dependencies(lap_time_server
  rclcpp
  rclcpp_action
  navigation_robot_custom_interfaces
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)
add_executable(lap_time_client src/lap_time_client.cpp)
ament_target_dependencies(lap_time_client
  rclcpp
  rclcpp_action
  navigation_robot_custom_interfaces
)


# Install targets
install(TARGETS
  robot_driver_node
  wall_finder_server
  lap_time_server
  lap_time_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
  DESTINATION ${PROJECT_NAME}/include/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


