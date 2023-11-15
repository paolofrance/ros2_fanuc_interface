import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('fanuc_ros_interface'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'fanuc_ros_interface',
        name = 'fanuc_ros_inteface',
        executable = 'ros2_robot_control.py',
        parameters = [config]
    )
    ld.add_action(node)
    return ld