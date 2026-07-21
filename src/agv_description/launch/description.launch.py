from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    model=PathJoinSubstitution([FindPackageShare('agv_description'),'urdf','agv.urdf.xacro'])
    return LaunchDescription([Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':Command([FindExecutable(name='xacro'),' ',model]),'use_sim_time':True}])])
