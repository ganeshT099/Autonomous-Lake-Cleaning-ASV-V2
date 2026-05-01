"""
ASV System Launch — single file, starts everything.
  ros2 launch asv_web system.launch.py

Auto-started on boot via:  sudo systemctl enable asv
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction


def generate_launch_description():
    rmw = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

    # Local relay server — starts first so ws_bridge/auto_pilot can connect
    local_server = Node(
        package="asv_web", executable="local_server", output="screen",
        respawn=True, respawn_delay=5.0)

    # Sensors — respawn so they retry if hardware not ready yet
    gps = Node(
        package="asv_gps", executable="gps_node", output="screen",
        respawn=True, respawn_delay=5.0)

    imu = Node(
        package="asv_imu", executable="imu_node", output="screen",
        respawn=True, respawn_delay=5.0)

    camera = Node(
        package="asv_camera", executable="camera_node", output="screen",
        respawn=True, respawn_delay=5.0)

    # Thruster — respawn (I2C may not be ready immediately at boot)
    thruster = Node(
        package="asv_thruster", executable="ardu_thruster", output="screen",
        respawn=True, respawn_delay=5.0)

    # Web bridge + autonomous pilot — 3s delay so local_server is ready first
    ws_bridge = TimerAction(period=3.0, actions=[
        Node(package="asv_web", executable="ws_bridge", output="screen",
             respawn=True, respawn_delay=5.0)
    ])
    autonomous_pilot = TimerAction(period=3.0, actions=[
        Node(package="asv_web", executable="autonomous_pilot", output="screen",
             respawn=True, respawn_delay=5.0)
    ])

    return LaunchDescription([
        rmw,
        local_server,
        gps,
        imu,
        camera,
        thruster,
        ws_bridge,
        autonomous_pilot,
    ])
