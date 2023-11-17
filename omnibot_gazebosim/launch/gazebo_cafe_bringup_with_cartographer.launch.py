from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, GroupAction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
import xacro
import os


def generate_launch_description():

    # Constants for paths to different files and folders
    robot_gazebo_pkg_name = 'omnibot_gazebosim'
    robot_gazebo_pkg_share = FindPackageShare(
        package=robot_gazebo_pkg_name).find(robot_gazebo_pkg_name)
    nav_file_path = os.path.join(
        robot_gazebo_pkg_share, "config", 'navigation_mppi.yaml')
    map_path = os.path.join(robot_gazebo_pkg_share, "config", 'cafemap.yaml')
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    omni_gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('omnibot_gazebosim'), 'launch')
    cartographer_conf_basename = 'omnibot_2d.lua'
    cartographer_resolution = 0.05

    use_sim = True

    rvizconf = os.path.join(robot_gazebo_pkg_share, 'config', 'nav2.rviz')
    translatotr = Node(
        package='omnibot_gazebosim',
        executable='cmdvel_translator',
        name='cmdvel_translator',
                output='screen')

    gazebolaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [omni_gazebo_launch_file_dir,
                '/gazebo_cafe_bringup_scan_matcher.launch.py'],
        ),
        launch_arguments={
            'use_sim_time': str(use_sim),
            'rviz_file_path': rvizconf,
        }.items()
    )

    cartographergrid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
        arguments=['-resolution', str(cartographer_resolution)])

    imunormalizer = Node(
        package='omnibot_gazebosim',
        executable='imu_quat_normalizer',
        name='imu_quat_normalizer',
                output='screen')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
        remappings=[('/scan', '/omnibot/scan'),
                    ('/imu', '/omnibot/imu/normalized'),
                    ('/odom', '/odometry/filtered')],
        arguments=['-configuration_directory', os.path.join(robot_gazebo_pkg_share, 'config'),
                   '-configuration_basename', cartographer_conf_basename])

    return LaunchDescription([
        gazebolaunch,
        translatotr,
        # cartographer,
        # cartographergrid,
        imunormalizer
    ])
