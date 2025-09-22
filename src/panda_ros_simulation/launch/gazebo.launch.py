import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'panda_ros_simulation'
    pkg_share = get_package_share_directory(pkg_name)

    # Declare launch arguments
    xacro_file = LaunchConfiguration('xacro_file')
    world_file = LaunchConfiguration('world_file')

    declare_xacro_file_cmd = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro'),
        description='Absolute path to URDF/Xacro file'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='/usr/share/gazebo-11/worlds/empty.world',
        
        description='Absolute path to Gazebo world file'
    )

    # Process xacro safely
    xacro_path = os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")

    try:
        doc = xacro.process_file(xacro_path)
    except Exception as e:
        raise RuntimeError(f"Failed to process xacro file: {xacro_path}\nError: {e}")

    robot_description = {'robot_description': doc.toxml()}

    # Launch Gazebo server and client
    gzserver = ExecuteProcess(
        cmd=['gzserver', LaunchConfiguration('world_file'), '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'franka_panda', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        declare_xacro_file_cmd,
        declare_world_file_cmd,
        gzserver,
        gzclient,
        robot_state_pub,
        spawn_entity
    ])
