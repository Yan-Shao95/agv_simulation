from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items())

def generate_launch_description():
    vehicle_type = LaunchConfiguration('vehicle_type')
    use_nav2_rviz = LaunchConfiguration('use_nav2_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_type', default_value='differential_swerve'),
        DeclareLaunchArgument('use_nav2_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_navigation'), 'maps',
                'warehouse.yaml'])),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_navigation'), 'config', 'nav2.yaml'])),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_navigation'), 'config',
                'nav2_localization.rviz'])),
        include('agv_bringup', 'sim.launch.py', {
            'vehicle_type': vehicle_type,
            'use_rviz': 'false',
        }),
        include('agv_navigation', 'navigation.launch.py', {
            'map': map_file,
            'params_file': params_file,
        }),
        TimerAction(
            period=5.0,
            actions=[ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/drive/set_control_mode',
                    'agv_interfaces/srv/SetControlMode', '{mode: 1}',
                ],
                condition=IfCondition(PythonExpression([
                    "'", vehicle_type, "' == 'differential_swerve'",
                ])),
                output='screen',
            )],
        ),
        Node(
            package='rviz2', executable='rviz2',
            name='nav2_rviz', arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_nav2_rviz), output='screen'),
    ])
