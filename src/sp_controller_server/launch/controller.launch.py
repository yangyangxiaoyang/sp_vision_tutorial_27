from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sp_controller_server",
            executable="controller_node",
            name="controller_server",
            output="screen",
            parameters=[{
                "local_path_topic": "global_path",
                "odom_topic": "/Odometry",
                "cmd_vel_topic": "/sentry/cmd_vel",
                "base_frame_id": "base_link",
                "control_frequency": 50.0,
                "speed_limit": 5.0,

                "plugin_name": "PidController",
                "plugin_type": "PidController",

                "PidController.max_linear_speed": 0.8,
                "PidController.max_angular_speed": 1.5,
                "PidController.lookahead_dist": 0.6,
                "PidController.kp": 1.0,
                "PidController.ki": 0.0,
                "PidController.kd": 0.2,
                "PidController.distance_deadband": 0.02,
                "PidController.goal_slowdown_dist": 0.5,
                "PidController.goal_stop_dist": 0.05,
                "PidController.base_frame_id": "base_link",
            }]
        )
    ])
