import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('sp_nav_bt')
    default_bt_xml_path = os.path.join(pkg_dir, 'nav_bt', 'default_nav_with_fallback.xml')

    bt_xml_arg = DeclareLaunchArgument(
        'default_bt_xml',
        default_value=default_bt_xml_path,
        description='Full path to the behavior tree XML file'
    )

    bt_plugins_arg = DeclareLaunchArgument(
        'bt_plugins',
        default_value='['
            'nav_compute_path_service,'
            'nav_state_track,'
            'nav_rate_control,'
            'nav_repeat_sequence,'
            'nav_get_pose,'
            'nav_set_control_enable'
            ']',
        description='List of behavior tree plugin libraries to load'
    )

    bt_cout_logger_arg = DeclareLaunchArgument(
        'enable_bt_cout_logger',
        default_value='false',
        description='Enable BT::StdCoutLogger'
    )

    nav_interface_node = Node(
        package='sp_nav_bt',
        executable='nav_interface_node',
        name='nav_interface_node',
        output='screen',
        parameters=[{
            'default_bt_xml': LaunchConfiguration('default_bt_xml'),
            'bt_plugins': LaunchConfiguration('bt_plugins'),
            'enable_bt_cout_logger': LaunchConfiguration('enable_bt_cout_logger'),
        }]
    )

    return LaunchDescription([
        bt_xml_arg,
        bt_plugins_arg,
        bt_cout_logger_arg,
        nav_interface_node
    ])
