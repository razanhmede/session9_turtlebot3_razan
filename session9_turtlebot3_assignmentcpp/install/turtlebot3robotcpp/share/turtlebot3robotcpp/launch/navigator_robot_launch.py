from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Include the TurtleBot3 DQN stage 1 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_gazebo'),
                '/launch/turtlebot3_dqn_stage1.launch.py'
            ])
        ),

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
        # Node for the LapTime server
        Node(
            package='turtlebot3robotcpp',
            executable='lap_time_server',
            name='lap_time_server',
            output='screen',
        ),
        # Node for the LapTime client
        Node(
            package='turtlebot3robotcpp',
            executable='lap_time_client',
            name='lap_time_client',
            output='screen',
        ),
    ])




        

