import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Package and file paths
    pkg_name = 'panda_ros_simulation'
    my_robot_pkg = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(my_robot_pkg, 'urdf', 'panda.urdf.xacro')
    controller_file = os.path.join(my_robot_pkg, 'config', 'controller.yaml')

    # --- Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Robot Description ---
    robot_description = Command(['xacro ', urdf_file])

    set_prime_offload = SetEnvironmentVariable(
    name='__NV_PRIME_RENDER_OFFLOAD',
    value='1'
)

    set_glx_vendor = SetEnvironmentVariable(
    name='__GLX_VENDOR_LIBRARY_NAME',
    value='nvidia'
)
    # --- Gazebo Sim (Ignition) ---
    # We now use 'ros_gz_sim' and the 'empty.sdf' world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # --- Nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawner for Gazebo Sim (Ignition)
    # We now use 'ros_gz_sim' and its 'create' executable
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'franka_panda',
                   '-allow_renaming', 'true'],
        output='screen'
    )

    # --- ros2_control Controllers ---
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controller_file],
    )
    
    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', 'eef_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controller_file],
    )

    # --- Launch Description ---
    return LaunchDescription([
        declare_use_sim_time_cmd,
        set_prime_offload,
        set_glx_vendor,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        
        # Use an event handler to spawn controllers *after* the robot is spawned
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