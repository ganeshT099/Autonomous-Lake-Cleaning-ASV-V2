from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_thruster'), 'config', 'thruster_params.yaml')
    return LaunchDescription([
        Node(
            package='asv_thruster',
            executable='asv_thruster_node',
            name='asv_thruster_node',
            output='screen',
            parameters=[config],
        )
    ])
