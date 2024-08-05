
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
import xacro
from launch_ros.actions import SetParameter


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="crx20ia_l",
            description="model of the fanuc robot. ",
        )
    )

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

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz",
            default_value="false",
            description="If mock hardware, simulate using Gazebo ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_headless",
            default_value="true",
            description="Use Gazebo in headless mode",
        )
    )
    
    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)] )

def launch_setup(context, *args, **kwargs):

    description_package = "crx_description"    
    robot_type = LaunchConfiguration("robot_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    read_only = LaunchConfiguration("read_only")
    use_rmi = LaunchConfiguration("use_rmi")
    gz = LaunchConfiguration("gz")
    gz_headless = LaunchConfiguration("gz_headless")

    robot_type_str = robot_type.perform(context)
    print("robot_type", robot_type_str)



    # use_sim_time
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('gz'))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf", robot_type_str, robot_type_str+".xacro"]),
            " ", "robot_type:=", robot_type,
            " ", "use_mock_hardware:=", use_mock_hardware,
            " ", "robot_ip:=", robot_ip,
            " ", "read_only:=", read_only,
            " ", "use_rmi:=", use_rmi,
            " ", "gz:=", gz
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
            FindPackageShare("fanuc_control"),
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
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_type_str+'_moveit_config'),'launch', 'move_group.launch.py')]),
        launch_arguments=[("robot_type", robot_type)]
    )
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_type_str+'_moveit_config'),'launch', 'moveit_rviz.launch.py')]),)
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx_description'),'launch', 'gazebo.launch.py')]),
        launch_arguments={"headless" : gz_headless}.items(), 
        condition=IfCondition(gz))

    nodes_to_start = [
        set_use_sim_time,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
        gazebo_launch
    ]

    return nodes_to_start
