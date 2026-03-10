from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_gps'), 'config', 'gps_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='asv_gps',
            executable='asv_gps_node',
            name='asv_gps_node',
            output='screen',
            parameters=[config],
        )
    ])
