from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description(): return LaunchDescription([Node(package='agv_drive_controller',executable='swerve_kinematics',parameters=[{'use_sim_time':True}])])
