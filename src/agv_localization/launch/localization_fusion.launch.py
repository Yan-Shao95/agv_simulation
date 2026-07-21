from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    use_gps=LaunchConfiguration('use_gps')
    ekf=PathJoinSubstitution([FindPackageShare('agv_localization'),'config','ekf.yaml'])
    navsat=PathJoinSubstitution([FindPackageShare('agv_localization'),'config','navsat_transform.yaml'])
    return LaunchDescription([DeclareLaunchArgument('use_gps',default_value='false'),Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node',parameters=[ekf,{'use_sim_time':True}],remappings=[('odometry/filtered','/odometry/filtered')]),Node(package='robot_localization',executable='navsat_transform_node',name='navsat_transform',condition=IfCondition(use_gps),parameters=[navsat],remappings=[('imu','/sensors/imu/data'),('gps/fix','/sensors/gps/fix'),('odometry/filtered','/odometry/filtered')])])
