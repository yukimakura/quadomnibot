from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro
import os


def generate_launch_description():
    pkgname = "omni3wdbot_description"
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(pkgname), "urdf", "omni3wdbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    rvizconf = os.path.join(FindPackageShare(package=pkgname).find(pkgname), "rviz", 'model.rviz')

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        )
    
    rviznode = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rvizconf],
            output='screen')
    
    return LaunchDescription(
        [
            node_robot_state_publisher,
            node_joint_state_publisher,
            rviznode
        ]
    )
