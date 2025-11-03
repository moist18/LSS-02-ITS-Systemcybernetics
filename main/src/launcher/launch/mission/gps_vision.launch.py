import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mission_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'mission',
                        'gps_vision.yaml'
                    )

    monitoring_param = os.path.join(
                        get_package_share_directory('launcher'),
                        'config',
                        'monitoring.yaml'
                    )

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
                    FindPackageShare('launcher'), 'launch', 'rosbridge.launch.py'
                ])
            ])
        ),


        # MISSION PACKAGE
        
        Node(
            package='mission',
            executable='gps_vision_planner',
            name='planner_node',
            output='screen',
            emulate_tty=True,
            parameters=[mission_param]
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
        ),
        
        
        # NAVIGATION PACKAGE
        
        Node(
            package='navigation',
            executable='gps_nav',
            name='gps_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[navigation_param]
        ),
        Node(
            package='navigation',
            executable='vision_nav',
            name='vision_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[navigation_param]
        ),
        
        
        # PERCEPTION PACKAGE
        
        Node(
            package='perception',
            executable='underwater',
            name='underwater_node',
            output='screen',
            emulate_tty=True,
            parameters=[perception_param]
        ),
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
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[sensor_param]
        ),
        Node(
            package='sensor',
            executable='rplidar',
            name='lidar_node',
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