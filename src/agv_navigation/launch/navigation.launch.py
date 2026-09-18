from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_navigation'), 'maps',
                'warehouse.yaml'])),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_navigation'), 'config', 'nav2.yaml'])),
        DeclareLaunchArgument('autostart', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch',
                'bringup_launch.py'])),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': autostart,
                'slam': 'False',
                'use_localization': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items()),
    ])
