from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('asv_joystick'),
        'config', 'joystick_params.yaml')
    return LaunchDescription([
        # joy driver node — reads controller hardware
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        # ASV joystick mapper node
        Node(
            package='asv_joystick',
            executable='asv_joystick_node',
            name='asv_joystick_node',
            output='screen',
            parameters=[config],
        ),
    ])
