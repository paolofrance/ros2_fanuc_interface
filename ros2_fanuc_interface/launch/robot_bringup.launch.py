
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

    #declared_arguments.append(
    #    DeclareLaunchArgument(
    #        "robot_model",
    #        default_value="crx20ia_l",
    #        description="model of the fanuc robot. ",
    #    )
    #)

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
            "controllers_file",
            default_value="controllers.yaml"
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
            "use_rmi",
            default_value="false",
            description="if the robot is read only . ",
        )
    )

    description_package = "crx_description"
    #robot_model = LaunchConfiguration("robot_model")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    read_only = LaunchConfiguration("read_only")
    use_rmi = LaunchConfiguration("use_rmi")
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/crx20ia_l/", "crx20ia_l.xacro"]),
            " ", "use_mock_hardware:=", use_mock_hardware,
            " ", "robot_ip:=", robot_ip,
            " ", "read_only:=", read_only,
            " ", "use_rmi:=", use_rmi,
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
            controllers_file,
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
    )
    
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx20_moveit_config'),'launch', 'move_group.launch.py')]),)
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx20_moveit_config'),'launch', 'moveit_rviz.launch.py')]),)
    
    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
    ]

    return LaunchDescription( declared_arguments + nodes_to_start )



