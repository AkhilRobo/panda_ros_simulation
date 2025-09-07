from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('panda_ros_simulation')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'panda.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    ])



# Terminal comands 

# Terminal 1:
# step 1(converting xacro to urdf file)

# $: ros2 run xacro xacro $(ros2 pkg prefix panda_simulation)/share/panda_simulation/urdf/panda.urdf.xacro > panda.urdf

# step 2(now we have to provide the generated urdf in step 1 as input to robot_state_publisher)

# $: ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat panda.urdf)"



# terminal 2:

# in terminal 1 all the urdf data is being published , if we open rviz we can see our robot.

# $: ros2 run rviz2 rviz2