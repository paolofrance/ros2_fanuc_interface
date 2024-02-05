
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
import xacro

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "wrench_topic",
            default_value="/robotiq_ft_wrench",
        )
    )
    wrench_topic = LaunchConfiguration("wrench_topic")
    
    dpm_node = Node(
        package="ros2_fanuc_interface",
        executable="dpm",
        parameters=[
            {"wrench_topic": wrench_topic}
        ]
    )
    
    nodes_to_start = [
        dpm_node,
    ]

    return LaunchDescription( declared_arguments + nodes_to_start )



