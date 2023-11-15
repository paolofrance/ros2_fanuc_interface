import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ros2_fanuc_interface'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'ros2_fanuc_interface',
        name = 'ros2_fanuc_interface',
        executable = 'ros2_fanuc_interface.py',
        parameters = [config]
    )
    ld.add_action(node)
    return ld