from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    world=PathJoinSubstitution([FindPackageShare('agv_worlds'),'worlds','warehouse_40x50.sdf'])
    description=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('agv_description'),'launch','description.launch.py'])))
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'),'launch','gz_sim.launch.py'])),launch_arguments={'gz_args':['-r ',world]}.items())
    spawn=Node(package='ros_gz_sim',executable='create',arguments=['-topic','robot_description','-name','agv','-x','0','-y','-20','-z','0.3'])
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':PathJoinSubstitution([FindPackageShare('agv_gazebo'),'config','bridge.yaml'])}])
    joint_states=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager','/controller_manager'])
    wheel_velocity=Node(package='controller_manager',executable='spawner',arguments=['wheel_velocity_controller','--controller-manager','/controller_manager'])
    return LaunchDescription([gazebo,description,spawn,bridge,joint_states,wheel_velocity])
