from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('master_controller'),
        'config',
        'runtime_config.yaml'
    )
    return LaunchDescription([

        Node(
            package='master_controller',
            executable='stepwise_master_controller_charuco_node',
            name='master_controller_node',
            output='screen',
            parameters=[config]
        ),
    ])