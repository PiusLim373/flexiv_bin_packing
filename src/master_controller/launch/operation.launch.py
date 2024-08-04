from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    master_controller_config_file = os.path.join(
        get_package_share_directory("master_controller"), "config/runtime_config.yaml"
    )
    rs2_launch_file_dir = PathJoinSubstitution(
        [
            FindPackageShare("realsense2_camera"),
            "launch",
        ]
    )
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs2_launch_file_dir, "/rs_launch.py"]),
        launch_arguments={
            "pointcloud.enable": "true",
            "depth_module.profile": "1280x720x30",
            "rgb_camera.profile": "1280x720x30",
            "pointcloud.ordered_pc": "true",
            "align_depth.enable": "true",
        }.items(),
    )
    master_controller_node = Node(
        package="master_controller",
        executable="master_controller_node",
        name="master_controller",
        parameters=[master_controller_config_file],
    )

    return LaunchDescription([
        camera_node,
        # master_controller_node
        ])
