import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    perception_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'perception.yaml'
                    )
    
    return LaunchDescription([
        
        # PERCEPTION PACKAGE
        
        # Node(
        #     package='perception',
        #     executable='underwater',
        #     name='underwater_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[perception_param]
        # ),
        
        Node(
            package='perception',
            executable='vision.py',
            name='vision_node',
            output='screen',
            emulate_tty=True,
            parameters=[perception_param]
        ),
    ])