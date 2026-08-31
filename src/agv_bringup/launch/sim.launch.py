from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def include(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(), condition=condition)

def generate_launch_description():
    vehicle_type = LaunchConfiguration('vehicle_type')
    is_differential = IfCondition(PythonExpression([
        "'", vehicle_type, "' == 'differential_swerve'"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_type', default_value='differential_swerve',
            description='differential_swerve、mecanum 或 ackermann'),
        include('agv_gazebo', 'simulation.launch.py',
                {'vehicle_type': vehicle_type}),
        include('agv_drive_controller', 'control_chain.launch.py',
                condition=is_differential),
    ])
