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
    robot_gazebo_pkg_name = 'omni3wdbot_gazebosim'
    robot_gazebo_pkg_share = FindPackageShare(
        package=robot_gazebo_pkg_name).find(robot_gazebo_pkg_name)
    nav_file_path = os.path.join(
        robot_gazebo_pkg_share, "config", 'navigation_mppi.yaml')
    map_path = os.path.join(robot_gazebo_pkg_share, "config", 'cafemap.yaml')
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    omni_gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('omni3wdbot_gazebosim'), 'launch')

    rvizconf = os.path.join(robot_gazebo_pkg_share, 'config', 'nav2.rviz')
    translatotr = Node(
        package='omni3wdbot_gazebosim',
        executable='cmdvel_translator',
        name='cmdvel_translator',
                output='screen')

    nav2launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_launch_file_dir, '/bringup_launch.py'],
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'True',
            'params_file': nav_file_path,
        }.items()
    )

    gazebolaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [omni_gazebo_launch_file_dir, '/gazebo_cafe_bringup_scan_matcher.launch.py'],
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'rviz_file_path': rvizconf,
        }.items()
    )

    return LaunchDescription([
        nav2launch,
        gazebolaunch,
        translatotr
    ])
