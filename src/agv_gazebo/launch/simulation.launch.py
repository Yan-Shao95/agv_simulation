from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import re
import subprocess
import tempfile
import xacro


def _mecanum_sdf_from_xacro(model_path):
    urdf = xacro.process_file(str(model_path)).toxml()
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as model_file:
        model_file.write(urdf)
        model_file.flush()
        result = subprocess.run(
            ['gz', 'sdf', '-p', model_file.name], check=True,
            capture_output=True, text=True)
    return re.sub(
        r'<fdir1>([^<]+)</fdir1>',
        r"<fdir1 gz:expressed_in='base_footprint'>\1</fdir1>",
        result.stdout)


def _spawn_vehicle(context, vehicle_type):
    selected = vehicle_type.perform(context)
    common = ['-name', 'agv', '-x', '0', '-y', '-20', '-z', '0.35']
    if selected == 'mecanum':
        model_path = Path(get_package_share_directory('agv_description')) / 'urdf' / 'mecanum.urdf.xacro'
        return [Node(package='ros_gz_sim', executable='create',
                     arguments=['-string', _mecanum_sdf_from_xacro(model_path), *common])]
    return [Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', *common])]

def generate_launch_description():
    vehicle_type = LaunchConfiguration('vehicle_type')
    world = PathJoinSubstitution([FindPackageShare('agv_worlds'), 'worlds', 'warehouse_40x50.sdf'])
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('agv_description'), 'launch', 'description.launch.py'])),
        launch_arguments={'vehicle_type': vehicle_type}.items())
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world]}.items())
    spawn = OpaqueFunction(function=_spawn_vehicle, args=[vehicle_type])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': PathJoinSubstitution([
                      FindPackageShare('agv_gazebo'), 'config', 'bridge.yaml'])}])
    is_differential = IfCondition(PythonExpression([
        "'", vehicle_type, "' == 'differential_swerve'"]))
    joint_states = Node(package='controller_manager', executable='spawner',
                        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                        condition=is_differential)
    wheel_velocity = Node(package='controller_manager', executable='spawner',
                          arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
                          condition=is_differential)
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_type', default_value='differential_swerve'),
        gazebo, description, spawn, bridge, joint_states, wheel_velocity,
    ])
