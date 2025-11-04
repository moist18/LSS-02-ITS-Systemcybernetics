import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    monitoring_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'monitoring.yaml'
                    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launcher'), 'launch', 'mavros.launch.py'
                ])
            ])
        ),
        
        # MONITORING PACKAGE
        
        Node(
            package='monitoring',
            executable='battery',
            name='battery_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='monitoring',
            executable='cpu',
            name='cpu_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='monitoring',
            executable='date',
            name='date_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='monitoring',
            executable='gps',
            name='gps_node',
            output='screen',
            emulate_tty=True
        )
    ])
