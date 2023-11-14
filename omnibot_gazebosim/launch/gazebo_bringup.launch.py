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

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_path = LaunchConfiguration('world_file_path', default='')
    rvizconf = LaunchConfiguration('rviz_file_path', default='')
    # Pose where we want to spawn the robot
    spawn_x_val = LaunchConfiguration('spawn_x_value', default='1.0')
    spawn_y_val = LaunchConfiguration('spawn_y_value', default='1.0')
    spawn_z_val = LaunchConfiguration('spawn_z_value', default='1.0')
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_value', default='0.0')

    # Constants for paths to different files and folders
    robot_pkg_name = 'omnibot_description'
    robot_gazebo_pkg_name = 'omnibot_gazebosim'
    robot_name_in_model = 'omnibot'
    urdf_file_path = 'urdf/omnibot.urdf.xacro'


    robot_gazebo_pkg_share = FindPackageShare(
        package=robot_gazebo_pkg_name).find(robot_gazebo_pkg_name)
    robot_pkg_share = FindPackageShare(
        package=robot_pkg_name).find(robot_pkg_name)

    ekf_path = os.path.join(robot_gazebo_pkg_share, "config", 'ekf_config.yaml')
    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items(),
    )

    urdf_model_path = os.path.join(robot_pkg_share, urdf_file_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    gazebo_robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnibot_base_controller",
                   "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    jointstatepub = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name_in_model,
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val,
                                   ],
                        output='screen')

    ekfnode = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_path, {'use_sim_time': use_sim_time}])
    
    rviznode = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rvizconf],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    nodes = [
        # Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[rviznode],
            )
        ),
    
        gazebo,
        gazebo_robot_state_pub_node,
        jointstatepub,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        ekfnode

    ]

    return LaunchDescription(nodes)
