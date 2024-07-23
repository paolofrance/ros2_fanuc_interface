from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="crx20ia_l",
            description="model of the fanuc robot. ",
        )
    )

    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)] )

def launch_setup(context, *args, **kwargs):
    
    robot_type = LaunchConfiguration("robot_type")
    robot_type_str = robot_type.perform(context)
    print("robot_type", robot_type_str)

    description_package = "crx_description"
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/"+robot_type_str+"/", robot_type_str+".xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}    
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "config", "config.rviz"])

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return nodes_to_start
