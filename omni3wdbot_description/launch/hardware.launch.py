from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os


def generate_launch_description():

    rvizconf = LaunchConfiguration('rviz_file_path', default='')
    # Pose where we want to spawn the robot
    spawn_x_val = LaunchConfiguration('spawn_x_value', default='1.0')
    spawn_y_val = LaunchConfiguration('spawn_y_value', default='1.0')
    spawn_z_val = LaunchConfiguration('spawn_z_value', default='1.0')
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_value', default='0.0')
    ekf_path = LaunchConfiguration('ekf_param_path', default='')

    # Constants for paths to different files and folders
    robot_pkg_name = 'omni3wdbot_description'
    robot_name_in_model = 'omni3wdbot'
    urdf_file_path = 'urdf/omni3wdbot.urdf.xacro'


    robot_pkg_share = FindPackageShare(
        package=robot_pkg_name).find(robot_pkg_name)

    controller_config_path =  os.path.join(robot_pkg_share, 'config','omni3wdbot_controllers.yaml')
    
    urdf_model_path = os.path.join(robot_pkg_share, urdf_file_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c" , "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni3wdbot_base_controller", "-c" , "/controller_manager"],
            output="screen",
        )

    robot_controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": doc.toxml()}, controller_config_path],
            output="screen",
        )


    jointstatepub = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            )


    # ekfnode = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_path, {'use_sim_time': use_sim_time}])
    
    rviznode = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rvizconf],
            # parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    nodes = [
        robot_controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        robot_state_pub_node,
        jointstatepub,
        # ekfnode,
        # rviznode

    ]

    return LaunchDescription(nodes)