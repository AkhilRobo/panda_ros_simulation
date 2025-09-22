import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'panda_ros_simulation'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to the default RViz configuration and URDF model
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')

    # --- Launch Arguments ---
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )

    # --- Robot Description ---
    # Use ParameterValue to process the xacro command and treat its output as a string
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', urdf_file_path]),
        value_type=str
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            # Use simulation time when launched with Gazebo
            'use_sim_time': True 
        }]
    )

    # --- RViz Node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])
