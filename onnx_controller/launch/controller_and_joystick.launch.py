from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onnx_controller',
            executable='controller',
            name='onnx_controller_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[]
        )
    ])
