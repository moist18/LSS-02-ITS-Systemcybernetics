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
    
    return LaunchDescription([
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
    ])