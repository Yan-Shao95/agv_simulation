from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def include(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(), condition=condition)

def generate_launch_description():
    vehicle_type = LaunchConfiguration('vehicle_type')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    is_differential = IfCondition(PythonExpression([
        "'", vehicle_type, "' == 'differential_swerve'"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_type', default_value='differential_swerve',
            description='differential_swerve、mecanum 或 ackermann'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 实时显示机器人和激光扫描点'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('agv_bringup'), 'config', 'simulation.rviz']),
            description='RViz2 配置文件'),
        include('agv_gazebo', 'simulation.launch.py',
                {'vehicle_type': vehicle_type}),
        include('agv_drive_controller', 'control_chain.launch.py',
                condition=is_differential),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
