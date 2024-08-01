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
    ])


        

