import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'panda_ros_simulation'
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'panda.urdf.xacro'
    )

    # Process the xacro file
    from xacro import process_file
    doc = process_file(urdf_file)
    robot_description = doc.toxml()

    # Launch Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Launch Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Launch Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn Panda in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'franka_panda',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_pub,
        spawn_entity
    ])
