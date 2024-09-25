from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('vision_server'),
        'config',
        'box_finder.yaml'
    )
    return LaunchDescription([

        Node(
            package='vision_server',
            executable='box_finder_node',
            name='box_finder_node',
            output='screen',
            parameters=[config]
        ),
    ])