import os

from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler,DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess, OpaqueFunction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from ament_index_python.packages import get_package_share_directory


ARGUMENTS =[ 
    DeclareLaunchArgument('name',  default_value = '',     description = 'NAME_SPACE'     ),
    DeclareLaunchArgument('model', default_value = 'crx10ia_l',     description = 'ROBOT_MODEL'    ),
    DeclareLaunchArgument('headless',   default_value = 'true',     description = 'Headless Gazebo'    ),
    DeclareLaunchArgument('x',   default_value = '0',     description = 'Location x on Gazebo '    ),
    DeclareLaunchArgument('y',   default_value = '0',     description = 'Location y on Gazebo'    ),
    DeclareLaunchArgument('z',   default_value = '0',     description = 'Location z on Gazebo'    ),
    DeclareLaunchArgument('R',   default_value = '0',     description = 'Location Roll on Gazebo'    ),
    DeclareLaunchArgument('P',   default_value = '0',     description = 'Location Pitch on Gazebo'    ),
    DeclareLaunchArgument('Y',   default_value = '0',     description = 'Location Yaw on Gazebo'    ),
    #DeclareLaunchArgument('use_sim_time',  default_value='true', description='If true, use simulated clock'), # Looks like its automatic
]

ENVIRONMENT = [SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.dirname(get_package_share_directory("crx_description")))]

def generate_launch_description():
    # Initialize Arguments
    # gui = LaunchConfiguration("gui")
    #use_sim_time = LaunchConfiguration('use_sim_time')


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("crx_description"),
                    "urdf",
                    LaunchConfiguration('model'),
                    LaunchConfiguration('model'),
                ]
            ),
            ".xacro",
            " ",
            "gz:=true",
            " "
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("crx10ia_l_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    ]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_manipulator_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'manipulator_controller'],
        output='screen'
    )


    # def invert_headless(context: LaunchContext, headless):
    # headless_bool = 
    # gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), '/launch/gazebo.launch.py']),
                launch_arguments = {"gui" : PythonExpression(["'", LaunchConfiguration('headless'), "'.lower() == 'false'"])}.items()
             )
    
    # ,
    #             launch_arguments = {"gui" : not(LaunchConfiguration('headless'))}.items()

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model. Done with ENVIRONMENT variable.
    gz_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[
                            "-topic",
                            robot_description,
                            "-entity",
                            LaunchConfiguration('model'),
                            "-x",
                            LaunchConfiguration('x'),
                            "-y",
                            LaunchConfiguration('y'),
                            "-z",
                            LaunchConfiguration('z'),
                            "-R",
                            LaunchConfiguration('R'),
                            "-P",
                            LaunchConfiguration('P'),
                            "-Y",
                            LaunchConfiguration('Y'),
                            "-timeout", "60",
                        ],
                        output='screen')
    

    return LaunchDescription(ENVIRONMENT + ARGUMENTS + [
        node_robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_manipulator_controller],
            )
        ),
    ])

