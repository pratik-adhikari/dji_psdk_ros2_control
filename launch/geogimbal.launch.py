from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geogimbal',
            executable='gimbal_controller',
            name='gimbal_controller',
            output='screen'
        ),
        Node(
            package='geogimbal',
            executable='psdk_interface',
            name='psdk_interface',
            output='screen'
        )
    ])
