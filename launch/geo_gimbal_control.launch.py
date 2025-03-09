# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='geogimbal',
#             executable='gimbal_controller',
#             name='gimbal_controller',
#             output='screen'
#         ),
#         Node(
#             package='geogimbal',
#             executable='psdk_interface',
#             name='psdk_interface',
#             output='screen'
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    import ament_index_python.packages
    pkg_share = ament_index_python.packages.get_package_share_directory('geogimbal')
    params_file = os.path.join(pkg_share, 'config', 'parameters.yaml')

    return LaunchDescription([
        Node(
            package='geogimbal',
            executable='geo_gimbal_main',   # <-- renamed here
            name='gimbal_transform_node',
            output='screen',
            parameters=[params_file]
        )
    ])
