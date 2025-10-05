import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def generate_launch_description():
    # Set use_sim_time for all nodes
    use_sim_time = {"use_sim_time": True}

    # Build the MoveIt configuration
    # We still need this to load the robot_description and other params
    moveit_config = (
        MoveItConfigsBuilder("franka_panda", package_name="panda_moveit_config")
        .to_moveit_configs()
    )

    # 1. Start the ros2_control node (THE MISSING NERVOUS SYSTEM)
    # This connects MoveIt's commands to the Gazebo simulation
    ros2_controllers_path = os.path.join(
        moveit_config.package_path, "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path, use_sim_time],
        output="screen",
    )

    # 2. Spawn the controllers that MoveIt will use
    controller_spawners = [
        "arm_controller",
        "eef_controller"
    ]
    spawner_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[spawner, "-c", "/controller_manager"],
        )
        for spawner in controller_spawners
    ]

    # 3. Start your testing node (YOUR CLIENT)
    tutorial_node = Node(
        package="panda_control",
        executable="testing",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )
    
    # 4. Start RViz for visualization
    rviz_config_file = os.path.join(
        moveit_config.package_path, "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            use_sim_time,
        ],
    )


    return LaunchDescription(
        [
            ros2_control_node,
            *spawner_nodes,
            tutorial_node,
            rviz_node,
        ]
    )