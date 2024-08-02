from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for the WallFinder service server
        Node(
            package='turtlebot3robotcpp',
            executable='wall_finder_server',
            name='wall_finder_server',
            output='screen',
        ),
        # Node for the RobotDriver
        Node(
            package='turtlebot3robotcpp',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
        ),
        # Node for the Laptime server
        Node(
            package='turtlebot3robotcpp',
            executable='lap_time_server',
            name='lap_time_server',
            output='screen',
        ),
        # Node for the Laptime client
        Node(
            package='turtlebot3robotcpp',
            executable='lap_time_client',
            name='lap_time_client',
            output='screen',
        ),
    ])


        

