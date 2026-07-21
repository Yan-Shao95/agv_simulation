from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    config=PathJoinSubstitution([FindPackageShare('agv_drive_controller'),'config','controller.yaml'])
    names=['command_arbiter','swerve_kinematics','module_angle_controller','differential_module_mixer','motor_command_bridge','swerve_odometry']
    return LaunchDescription([Node(package='agv_drive_controller',executable=n,name=n,parameters=[config,{'use_sim_time':True}]) for n in names])
