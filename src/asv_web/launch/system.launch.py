from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='asv_web',
            executable='camera_server',
            output='screen'
        ),

        Node(
            package='asv_web',
            executable='ws_bridge',
            output='screen'
        ),

    ])
