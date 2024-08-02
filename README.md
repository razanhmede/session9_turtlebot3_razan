# session9_turtlebot3_razan

## Description
This project contains a ROS 2 action server and client for measuring the time taken by a robot to complete a lap around an arena. The server tracks the robot's position and computes the elapsed time for a full lap. The client sends a goal to the server to start the lap time measurement and receives feedback and results. Additionally, it includes a `RobotDriver` node to control the robot's movement so it doesn't hit the closest wall found  and a `WallFinderServer` node to find the closest wall.

## Overview

This project includes:
- A ROS 2 action server (`lap_time_server.cpp`) that measures the lap time of a robot around the arena.
- A ROS 2 action client (`lap_time_client.cpp`) that initiates the lap time measurement and receives feedback and results.
- A `RobotDriver` node (`robot_driver_node.cpp`) that controls the robot's movement.
- A `WallFinder` node (`wall_finder_server.cpp`) that finds the closest wall to the robot.


This project also includes the corresponding header files for these source code files.

## Usage

1. Clone this repository into the `src` directory

2. Build the workspace

3. Source the workspace

4. Run the launch file:
```sh
    ros2 launch turtlebot3robotcpp navigator_robot_launch.py
```
After running this launch file, you will be able to launch both gazebo turtlebot3 simulation and the source code files of the project.   

