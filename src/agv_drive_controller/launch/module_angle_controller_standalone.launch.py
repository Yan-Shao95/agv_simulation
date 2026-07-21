from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description(): return LaunchDescription([Node(package='agv_drive_controller',executable='module_angle_controller',parameters=[{'use_sim_time':True,'kp':4.0,'max_steering_velocity':3.0}])])
