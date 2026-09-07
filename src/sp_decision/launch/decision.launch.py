"""
decision.launch.py
------------------
Launch sp_decision with click_nav.xml as the default tree.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    pkg_share = get_package_share_directory('sp_decision')
    click_nav = os.path.join(pkg_share, 'decision_trees', 'click_nav.xml')

    node = Node(
        package='sp_decision',
        executable='sp_decision_node',
        output='screen',
        parameters=[
            config,
            {'tree_files': [click_nav]},
        ],
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('sp_decision')
    default_config = os.path.join(pkg_share, 'config', 'decision_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the sp_decision YAML parameter file',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
