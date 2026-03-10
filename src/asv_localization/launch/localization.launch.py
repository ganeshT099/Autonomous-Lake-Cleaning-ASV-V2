from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_localization'),
        'config', 'localization_params.yaml')
    return LaunchDescription([
        Node(
            package='asv_localization',
            executable='asv_localization_node',
            name='asv_localization_node',
            output='screen',
            parameters=[config],
        )
    ])
