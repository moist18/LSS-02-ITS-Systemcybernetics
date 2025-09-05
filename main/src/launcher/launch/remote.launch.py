import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
        
        
        # PERCEPTION PACKAGE
        
        # Node(
        #     package='perception',
        #     executable='recorder',
        #     name='recorder_node',
        #     output='screen',
        #     emulate_tty=True
        # ),
        
        # Node(
        #     package='perception',
        #     executable='vision',
        #     name='vision_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[perception_param]
        # ),
        
        
        # SENSOR PACKAGE
        
        Node(
            package='sensor',
            executable='arduino',
            name='arduino_node',
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