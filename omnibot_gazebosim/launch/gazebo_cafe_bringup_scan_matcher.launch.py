from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    rvizconf = LaunchConfiguration('rviz_file_path', default='')

    # Constants for paths to different files and folders
    robot_gazebo_pkg_name = 'omnibot_gazebosim'
    world_file_path = 'config/cafe.world'
    # Pose where we want to spawn the robot
    spawn_x_val = '1.0'
    spawn_y_val = '1.0'
    spawn_z_val = '1.0'
    spawn_yaw_val = '0.0'

    robot_gazebo_pkg_share = FindPackageShare(
        package=robot_gazebo_pkg_name).find(robot_gazebo_pkg_name)
    world_path = os.path.join(robot_gazebo_pkg_share, world_file_path)
    ekf_path = os.path.join(robot_gazebo_pkg_share, "config", 'ekf_config_scan_odomadd.yaml')

    omni_gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('omnibot_gazebosim'), 'launch')
    
    if(rvizconf == ''):
        rvizconf = os.path.join(robot_gazebo_pkg_share, 'config', 'odom.rviz')

    gazebolaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [omni_gazebo_launch_file_dir, '/gazebo_bringup.launch.py'],
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file_path': world_path,
            'spawn_x_value': spawn_x_val,
            'spawn_y_value': spawn_y_val,
            'spawn_z_value': spawn_z_val,
            'spawn_yaw_value': spawn_yaw_val,
            'ekf_param_path': ekf_path,
            'rviz_file_path': rvizconf
            
        }.items()
    )

    laserscanodom = Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            parameters=[
                {'publish_tf': False},
                {'laser_frame': 'lidar_link'},
                {'publish_odom': 'laser_scan_odom'},
                # {'max_iterations': '15'},
                ],
            remappings=[
                ('/scan', '/omnibot/scan'),
            ],
            output='screen')

    return LaunchDescription([
        gazebolaunch,
        laserscanodom
        ])
