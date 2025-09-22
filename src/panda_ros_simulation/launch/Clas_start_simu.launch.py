import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_name = 'panda_ros_simulation'
    my_robot_pkg = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(my_robot_pkg, 'urdf', 'panda.urdf.xacro')
    controller_file = os.path.join(my_robot_pkg, 'config', 'controller.yaml')

    robot_description = Command(['xacro ', urdf_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'empty.world'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'franka_panda', '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 'eef_controller', '--controller-manager', '/controller_manager']
    )
    
    delay_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, robot_controller_spawner]
        )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_after_spawn
    ])
