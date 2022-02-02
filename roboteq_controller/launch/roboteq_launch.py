import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('roboteq_controller'),
        'config',
        'query.yaml'
        )
        
    node=Node(
        package = 'roboteq_controller',
        name = 'roboteq_controller_node',
        executable = 'roboteq_controller_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld