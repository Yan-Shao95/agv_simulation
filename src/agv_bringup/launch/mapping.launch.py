from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items())

def generate_launch_description():
    vehicle_type = LaunchConfiguration('vehicle_type')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_type', default_value='differential_swerve'),
        include('agv_bringup', 'sim.launch.py', {'vehicle_type': vehicle_type}),
        include('agv_navigation', 'mapping.launch.py'),
    ])
