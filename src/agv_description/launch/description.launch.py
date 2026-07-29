from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

VEHICLES = {
    'differential_swerve': 'agv.urdf.xacro',
    'mecanum': 'mecanum.urdf.xacro',
    'ackermann': 'ackermann.urdf.xacro',
}

def launch_setup(context):
    vehicle_type = LaunchConfiguration('vehicle_type').perform(context)
    if vehicle_type not in VEHICLES:
        raise RuntimeError(
            f'未知 vehicle_type={vehicle_type}，可选值: {", ".join(VEHICLES)}')
    model = os.path.join(
        get_package_share_directory('agv_description'), 'urdf', VEHICLES[vehicle_type])
    from launch.substitutions import Command, FindExecutable
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([FindExecutable(name='xacro'), ' ', model]),
            'use_sim_time': True,
        }])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_type', default_value='differential_swerve',
            description='differential_swerve、mecanum 或 ackermann'),
        OpaqueFunction(function=launch_setup),
    ])
