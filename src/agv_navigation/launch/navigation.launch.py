from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description(): return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'),'launch','bringup_launch.py'])),launch_arguments={'map':PathJoinSubstitution([FindPackageShare('agv_navigation'),'maps','warehouse.yaml']),'params_file':PathJoinSubstitution([FindPackageShare('agv_navigation'),'config','nav2.yaml']),'use_sim_time':'true'}.items())])
