from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('aif_debug_visualizer')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([share, 'config', 'dashboard.rviz'])),
        DeclareLaunchArgument('decode_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            additional_env={'AIF_DEBUG_DECODE_CONFIG': LaunchConfiguration('decode_config')},
            output='screen'),
    ])
