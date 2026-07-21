from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description(): return LaunchDescription([Node(package='slam_toolbox',executable='async_slam_toolbox_node',parameters=[PathJoinSubstitution([FindPackageShare('agv_navigation'),'config','slam_toolbox.yaml'])])])
