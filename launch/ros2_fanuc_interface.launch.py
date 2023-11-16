from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
       
    config_file = PathJoinSubstitution([FindPackageShare("ros2_fanuc_interface"), "config", "params.yaml"])

    node=Node(
        package = 'ros2_fanuc_interface',
        name = 'ros2_fanuc_interface',
        executable = 'ros2_fanuc_interface.py',
        parameters = [config_file],
        arguments=["--ros-args", "--log-level", "info"],
    )

    nodes_to_start = [
        node,    
        ]

    return LaunchDescription(nodes_to_start)
