import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    navigation_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'navigation',
                        'gps_vision.yaml'
                    )
    
    perception_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'perception.yaml'
                    )

    sensor_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'sensor.yaml'
                    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launcher'), 'launch', 'mavros.launch.py'
                ])
            ])
        ),
        
        
        # NAVIGATION PACKAGE
        
        Node(
            package='navigation',
            executable='vision_nav',
            name='vision_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[navigation_param]
        ),
        
        
        # PERCEPTION PACKAGE
        
        # Node(
        #     package='perception',
        #     executable='recorder.py',
        #     name='recorder_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        
        Node(
            package='perception',
            executable='vision.py',
            name='vision_node',
            output='screen',
            emulate_tty=True,
            parameters=[perception_param]
        ),
        
        
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
            output='screen',
            emulate_tty=True,
            parameters=[sensor_param]
        )
    ])