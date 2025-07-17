import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sensor_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'sensor.yaml'
                    )
    
    vision_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'vision.yaml'
                    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launcher'), 'launch', 'mavros.launch.py'
                ])
            ])
        ),
        
        
        # PERCEPTION PACKAGE
        
        Node(
            package='perception',
            executable='vision',
            name='vision_node',
            output='screen',
            emulate_tty=True,
            parameters=[vision_param]
        ),
        # Node(
        #     package='navigation',
        #     executable='record',
        #     name='record_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        
        
        # SENSOR PACKAGE
        
        Node(
            package='sensor',
            executable='remote',
            name='remote_node',
            output='screen',
            emulate_tty=True,
            parameters=[sensor_param]
        ),
        Node(
            package='sensor',
            executable='state',
            name='state_node',
            output='screen',
            emulate_tty=True
        )
    ])