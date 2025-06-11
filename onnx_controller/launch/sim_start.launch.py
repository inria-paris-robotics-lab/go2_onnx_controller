from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    watchdog_launch_file = PathJoinSubstitution([
        FindPackageShare('go2_control_interface'),
        'launch',
        'watchdog.launch.py'
    ])

    return LaunchDescription([
        Node(
            package='go2_simulation',
            executable='simulation_node',
            name='simulation_node',
            output='screen',
            parameters=[
                {
                    'simulator': "pybullet",
                }
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([watchdog_launch_file]),
            launch_arguments={
                'n_fails': "2",
                'freq': "10",
            }.items()
        ),
])
