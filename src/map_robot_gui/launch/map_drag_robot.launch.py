#!/usr/bin/env python3
"""
map_drag_robot.launch.py

启动地图拖拽仿真 GUI，默认使用 sp_nav_bringup 包内 RMUC 地图。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    map_yaml = LaunchConfiguration('map_yaml').perform(context)

    parameters = [config]
    if map_yaml:
        parameters.append({'map_yaml': map_yaml})

    node = Node(
        package='map_robot_gui',
        executable='map_drag_robot',
        name='map_drag_robot_node',
        output='screen',
        emulate_tty=True,
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('map_robot_gui')
    bringup_share = get_package_share_directory('sp_nav_bringup')
    default_config = os.path.join(pkg_share, 'config', 'map_drag_robot.yaml')
    default_map = os.path.join(bringup_share, 'maps', 'rmuc_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='YAML 配置文件绝对路径',
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map,
            description='地图 YAML 绝对路径',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
