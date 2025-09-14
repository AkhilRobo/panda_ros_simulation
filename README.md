# Franka Panda ROS 2 Simulation & Debugging Journey

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)

This repository contains a complete ROS 2 simulation package for the Franka Emika Panda collaborative robot arm. It serves as a launchpad for advanced robotics projects, providing a detailed and functional robot model in a Gazebo simulation environment.

More than just a simulation, this repository documents a real-world, late-night debugging session that navigated and solved a series of cascading errors. It stands as a practical guide to the common pitfalls and solutions encountered when building a robot description from scratch in ROS 2.

## Demo

Here is the final, working robot model being controlled in RViz.

![Panda Arm RViz Demo](media/panda_simulation.gif)

## Key Concepts Covered

This project is a practical application of several core ROS and robotics concepts:

-   **XACRO (XML Macros):** Used extensively to create a modular, reusable, and easy-to-read robot description (URDF).
-   **URDF (Unified Robot Description Format):** The final XML format that describes the robot's physical properties (links, joints, visuals, collisions).
-   **Transmissions:** The crucial `<transmission>` tags that link the abstract kinematic joints to physical actuators, enabling simulation with `ros2_control`.
-   **Virtual Links:** The use of non-physical links to create stable, standardized coordinate frames, such as a tool flange (`TCP0`), for clean attachment of end-effectors.
-   **`ros2_control`:** The modern ROS 2 framework for controller management, integrated here with Gazebo via `gazebo_ros2_control` for physics-based simulation.

## Installation & Usage

Follow these steps to build and run the simulation.

### Prerequisites

-   Ubuntu 22.04 LTS
-   ROS 2 Humble Hawksbill
-   Gazebo Simulator
-   ROS 2 Build and Control Tools:
    ```bash
    sudo apt update
    sudo apt install ros-dev-tools ros-humble-xacro ros-humble-ros2-control ros-humble-gazebo-ros2-control
    ```

### Build & Run Instructions

1.  **Create and navigate to your ROS 2 workspace:**
    ```bash
    mkdir -p ~/panda_ws/src
    cd ~/panda_ws/src
    ```

2.  **Clone this repository:**
    ```bash
    # Replace with your actual repository URL
    git clone [https://github.com/AkhilRobo/panda_ros_simulation.git](https://github.com/AkhilRobo/panda_ros_simulation.git)
    ```

3.  **Install dependencies:**
    Navigate to your workspace root, and let `rosdep` install any missing dependencies.
    ```bash
    cd ~/panda_ws
    rosdep install --from-paths src -y --ignore-src
    ```

4.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```

5.  **Source the workspace:**
    Remember to source the workspace in every new terminal you open.
    ```bash
    source install/setup.bash
    ```

6.  **Launch the Visualization:**
    This command starts RViz and the `joint_state_publisher` GUI.
    ```bash
    ros2 launch panda_ros_simulation display.launch.py
    ```
    -   In RViz, set the **Fixed Frame** to `base_link` or `panda_link0` to see the robot.

## Future Work / Roadmap

-   [ ] **MoveIt 2 Integration:** Add a complete MoveIt configuration for the Panda arm to enable advanced motion planning, collision checking, and inverse kinematics.
-   [ ] **Path Planning:** Implement and test various path planning algorithms available in MoveIt and OMPL.
-   [ ] **Perception:** Integrate a simulated depth camera (such as the Intel RealSense) into the Gazebo environment.
-   [ ] **Object Detection:** Develop a ROS 2 node for detecting, localizing, and publishing the poses of objects in the camera's field of view.
