import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Paths ---
    pkg_name = 'panda_ros_simulation'
    my_robot_pkg = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(my_robot_pkg, 'urdf', 'panda.urdf.xacro')
    controller_file = os.path.join(my_robot_pkg, 'config', 'controller.yaml')

    # --- Robot Description ---
    robot_description = Command(['xacro ', urdf_file])
    robot_description_param = {'robot_description': robot_description}

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # --- Spawn Entity ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'franka_panda', '-topic', 'robot_description'],
        output='screen'
    )

    # --- ros2_control node ---
    # This is the bridge between ros2_control and Gazebo's hardware interface
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_param, controller_file, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # --- Joint State Broadcaster ---
    # This publishes the robot's state from the simulation
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    # Delay the broadcaster until the robot is spawned
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        ros2_control_node,
        delay_broadcaster_after_spawn
    ])