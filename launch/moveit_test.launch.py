
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


    description_package = "crx_description"
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/crx20ia_l/", "crx20ia_l.xacro"]),
            " ", "use_mock_hardware:=", use_mock_hardware,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_fanuc_interface"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        # arguments=["forward_position_controller", "-c", "/controller_manager"],
    )
    controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        # arguments=["forward_position_controller", "-c", "/controller_manager"],
    )
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "config", "config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx20_moveit_config'),'launch', 'move_group.launch.py')]),)
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx20_moveit_config'),'launch', 'moveit_rviz.launch.py')]),)
    
    nodes_to_start = [
        # rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        # controller_spawner_stopped,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
    ]

    return LaunchDescription( declared_arguments + nodes_to_start )



