import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sp_nav_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'nav_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'rviz.rviz')
    map_yaml = os.path.join(bringup_dir, 'maps', 'rmuc_map.yaml')

    map_server_node = Node(
        package='sp_map_server',
        executable='esdf_map_publisher',
        name='esdf_map_publisher',
        output='screen',
        parameters=[params_file, {'map_yaml': map_yaml}]
    )

    global_planner_node = Node(
        package='sp_global_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    nav_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sp_nav_bt'),
                'launch', 'nav_interface.launch.py')
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        map_server_node,
        global_planner_node,
        nav_interface_launch,
        rviz_node
    ])
