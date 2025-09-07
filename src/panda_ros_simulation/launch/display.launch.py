from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('panda_ros_simulation')

    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'panda.urdf.xacro'])
    default_rviz_config = PathJoinSubstitution([pkg, 'rviz', 'panda.rviz'])


    model_arg = DeclareLaunchArgument(
        'model', default_value=xacro_file,
        description='Path to the URDF/Xacro file'
    )
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_config,
        description='Path to the RViz config file'
    )

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
