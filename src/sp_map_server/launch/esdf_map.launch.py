from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sp_map_server",
            executable="esdf_map_publisher",
            name="esdf_map_publisher",
            output="screen",
            parameters=[{
                "map_yaml": "/home/rm/Desktop/sp_nav_26/src/tools/map_process/pgm/map.yaml",  # run-time override
                "frame_id": "map",
                "publish_rate_hz": 1.0,
                "unknown_as_obstacle": True,
                "robot_radius": 0.25,
                "margin": 0.10,
                "d_safe": 0.68,
                "w": 3.0,
                "unknown_cost": 100,
            }]
        )
    ])
