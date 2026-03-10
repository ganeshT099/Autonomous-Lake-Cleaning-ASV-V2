#!/usr/bin/env python3
"""
ASV Master Bringup Launch File
Launches all nodes with one command:
  ros2 launch asv_bringup asv_bringup.launch.py

Optional args:
  teleop:=true/false  (default: true)
  camera:=true/false  (default: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def cfg(pkg, filename):
    return os.path.join(get_package_share_directory(pkg), 'config', filename)

def generate_launch_description():

    # Launch arguments
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true',
        description='Launch keyboard teleop node')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Launch camera node')

    return LaunchDescription([
        teleop_arg,
        camera_arg,

        # ── GPS ──────────────────────────────────────────────
        Node(
            package='asv_gps',
            executable='asv_gps_node',
            name='asv_gps_node',
            output='screen',
            parameters=[cfg('asv_gps', 'gps_params.yaml')],
        ),

        # ── IMU ──────────────────────────────────────────────
        Node(
            package='asv_imu',
            executable='asv_imu_node',
            name='asv_imu_node',
            output='screen',
            parameters=[cfg('asv_imu', 'imu_params.yaml')],
        ),

        # ── Radar ─────────────────────────────────────────────
        Node(
            package='asv_radar',
            executable='asv_radar_node',
            name='asv_radar_node',
            output='screen',
            parameters=[cfg('asv_radar', 'radar_params.yaml')],
        ),

        # ── Ultrasonic ESP32 Bridge ───────────────────────────
        Node(
            package='asv_ultrasonic',
            executable='asv_ultrasonic_node',
            name='asv_ultrasonic_node',
            output='screen',
            parameters=[cfg('asv_ultrasonic', 'ultrasonic_params.yaml')],
        ),

        # ── Camera (optional) ─────────────────────────────────
        Node(
            package='asv_camera',
            executable='asv_camera_node',
            name='asv_camera_node',
            output='screen',
            parameters=[cfg('asv_camera', 'camera_params.yaml')],
            condition=IfCondition(LaunchConfiguration('camera')),
        ),

        # ── Thruster Control ──────────────────────────────────
        Node(
            package='asv_thruster',
            executable='asv_thruster_node',
            name='asv_thruster_node',
            output='screen',
            parameters=[cfg('asv_thruster', 'thruster_params.yaml')],
        ),

        # ── Keyboard Teleop (optional) ────────────────────────
        Node(
            package='asv_teleop',
            executable='asv_teleop_node',
            name='asv_teleop_node',
            output='screen',
            parameters=[cfg('asv_teleop', 'teleop_params.yaml')],
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
