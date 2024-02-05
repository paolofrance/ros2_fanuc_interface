
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
            "use_mock_hardware",
            default_value="false",
            description="False to use the standard mock of ros2",
            choices=["true", "false"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.11.31.111",
            description="the IP of the controlled robot",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "read_only",
            default_value="false",
            description="if the robot is read only . ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "read_only",
            default_value="false",
            description="if the robot is read only . ",
        )
    ) 
    
    description_package = "crx_description"
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    read_only = LaunchConfiguration("read_only")
    
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/crx20ia_l/", "crx20ia_l.xacro"]),
            " ", "use_mock_hardware:=", use_mock_hardware,
            " ", "robot_ip:=", robot_ip,
            " ", "read_only:=", read_only,
        ]
    )
    robot_description = {"robot_description": robot_description_content}    
    
    dpm_params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_fanuc_interface"),
            "config",
            "dpm_params.yaml",
        ]
    )
    
    
    dpm_node = Node(
        package="ros2_fanuc_interface",
        executable="cart_wrench_dpm",
        parameters=[robot_description, dpm_params],
    )
    
    nodes_to_start = [
        dpm_node,
    ]

    return LaunchDescription( declared_arguments + nodes_to_start )



