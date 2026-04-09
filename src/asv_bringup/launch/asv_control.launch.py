from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joystick_node = Node(
        package='asv_joystick',
        executable='asv_joystick_node',
        output='screen'
    )

    thruster_node = Node(
        package='asv_thruster',
        executable='asv_thruster_node',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        joystick_node,
        thruster_node
    ])
