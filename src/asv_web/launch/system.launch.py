from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        Node(package='asv_camera',   executable='camera_node',        output='screen'),
        Node(package='asv_gps',      executable='gps_node',           output='screen'),
        Node(package='asv_thruster', executable='asv_thruster_node',  output='screen'),
        Node(package='asv_web',      executable='ws_bridge',          output='screen'),

    ])
