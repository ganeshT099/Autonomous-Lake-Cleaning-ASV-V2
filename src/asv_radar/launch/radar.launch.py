from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_radar'), 'config', 'radar_params.yaml')
    return LaunchDescription([
        Node(
            package='asv_radar',
            executable='asv_radar_node',
            name='asv_radar_node',
            output='screen',
            parameters=[config],
        )
    ])
