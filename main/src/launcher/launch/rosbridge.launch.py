from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
                ])
            ]),
            launch_arguments={
                'port': '9090',
                'address': '0.0.0.0',
                'retry_startup_delay': '5.0',
                'fragment_timeout': '600',
                'delay_between_messages': '0',
                'max_message_size': '10000000',
                'unregister_timeout': '10.0'
            }.items()
        ),
        
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
            parameters=[{
                'topics_glob': '[*]',
                'services_glob': '[*]',
                'params_glob': '[*]'
            }]
        )
    ])
