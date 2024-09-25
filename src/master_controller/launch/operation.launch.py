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
            "depth_module.depth_profile": "1280x720x30",
            "depth_module.infra_profile": "1280x720x30",
            "rgb_camera.color_profile": "1280x720x30",
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

    vision_node = Node(
        package="vision_server",
        executable="charuco_reader.py",
        name="aruco_reader_node",
    )

    motion_server_node = Node(
        package="motion_server",
        executable="motion_server_node",
        name="motion_server_node",
    )

    world_tcp_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        parameters=[],
        arguments=["0.4", "0", "0.37", "3.14", "0", "3.14", "world", "tcp"],
    )

    tcp_camera_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["-0.06", "0", "-0.1", "0", "-1.57", "0", "tcp", "camera_link"],
    )
    
    camera_tf_handler_node = Node(
        package="vision_server",
        executable="camera_tf_handler.py",
        output="screen",
        arguments=[],
    )

    return LaunchDescription(
        [
            vision_node,
            camera_node,
            motion_server_node,
            # camera_tf_handler_node,
            # world_tcp_node,
            # tcp_camera_node,
            # master_controller_node
        ]
    )
