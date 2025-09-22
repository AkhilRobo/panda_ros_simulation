import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    pkg_name = 'panda_ros_simulation'
    my_robot_pkg = get_package_share_directory(pkg_name)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

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
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Note: The 'spawn_entity' executable is from the 'gazebo_ros' package, which is for Gazebo Classic.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'franka_panda', '-topic', 'robot_description'],
        output='screen'
    )

    # A separate node to publish the initial joint states from the URDF
    initial_joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn the controllers after the robot has been spawned
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controller_file],
    )
    
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 'eef_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controller_file],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        gazebo,
        robot_state_publisher,
        initial_joint_publisher,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    arm_controller_spawner,
                ]
            )
        )
    ])
