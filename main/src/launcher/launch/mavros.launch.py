from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mavros'), 'launch', 'apm.launch'
                ])
            ]),
            launch_arguments={'fcu_url': '/dev/ttyTHS1:57600'}.items()
        ),
        
        TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/mavros/param', 'SYSID_MYGCS', '1'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/mavros/param', 'WP_RADIUS', '2'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/mavros/param', 'WP_SPEED', '7'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/mavros/param', 'WP_ACCEL', '7'],
                    output='screen'
                )
            ]
        )
    ])