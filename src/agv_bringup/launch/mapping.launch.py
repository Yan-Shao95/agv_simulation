from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def inc(p,f): return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(p),'launch',f])))
def generate_launch_description(): return LaunchDescription([inc('agv_bringup','sim.launch.py'),inc('agv_navigation','mapping.launch.py')])
