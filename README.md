# Franka Panda ROS 2 Simulation & Debugging Journey

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)

This repository contains a complete ROS 2 simulation package for the Franka Emika Panda collaborative robot arm. It serves as a launchpad for advanced robotics projects, providing a detailed and functional robot model in a Gazebo simulation environment.

More than just a simulation, this repository documents a real-world, late-night debugging session that navigated and solved a series of cascading errors. It stands as a practical guide to the common pitfalls and solutions encountered when building a robot description from scratch in ROS 2.

## Demo

Here is the final, working robot model being controlled in RViz.

![Panda Arm RViz Demo](media/panda_simulation.gif)
*(To add your GIF, create a `media` folder, place `panda_demo.gif` inside it, and this link will work automatically.)*

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

## A Debugging Story: From Errors to Success

This repository was forged in a multi-hour debugging session. Here is a summary of the errors encountered and the lessons learned—a roadmap to save others from the same fate.

### Error 1: `material '...' is not unique`
-   **Symptom:** The `robot_state_publisher` node would crash immediately on launch.
-   **Cause:** A `<material>` tag with the same name was defined in two different XACRO files that were both included in the final model.
-   **Debug Process:** Used `grep -r 'material name="..."' src/` to find all files defining the material.
-   **Lesson:** All top-level URDF elements (`link`, `joint`, `material`) must have unique names. Define materials and other common properties in a single, central file and `<xacro:include>` it where needed. **Never define the same element twice.**

### Error 2: `link '...' is not unique`
-   **Symptom:** After fixing the material, a new parsing error appeared for a duplicate link.
-   **Cause:** The same as above, but for a `<link>`. A link was defined manually (`<link name="..."/>`) and was *also* created inside a macro that was called.
-   **Debug Process:** Manually reviewed the `panda.urdf.xacro` and `macros.urdf.xacro` files to find the two definitions.
-   **Lesson:** Understand what your macros are creating. If a macro's job is to create a link, do not also create that link manually.

### The "Impossible" Error: Fixes Not Working
-   **Symptom:** The `material is not unique` error persisted even after the source files were corrected.
-   **Cause:** This was a deep issue. The fixes were being made in the `src` directory, but the launch system was using old, cached files from the `install` directory because the package's `CMakeLists.txt` was not correctly configured to install the `urdf` directory on every build.
-   **Debug Process:**
    1.  Performed a clean build (`rm -rf build install log` then `colcon build`) in a **new terminal** to eliminate stale environments.
    2.  Verified that `CMakeLists.txt` contained the `install(DIRECTORY urdf ...)` command.
    3.  Manually processed the final URDF using `ros2 run xacro xacro ... > final_robot.urdf` to bypass the launch system and prove the logical error was still present in the XACRO files themselves.
-   **Lesson:** The `src` space is for editing, but the `install` space is what ROS runs. `colcon build` is the critical bridge between them. When facing "impossible" bugs, always **clean your workspace and build in a fresh terminal.**

### Error 3: `Two root links found: [...]`
-   **Symptom:** After all duplicates were fixed, the parser complained about the model's structure.
-   **Cause:** The URDF described two disconnected kinematic trees (one for the arm, one for the gripper). A valid URDF must be a single, contiguous tree.
-   **Debug Process:** The joint connecting the arm (`panda_link7`) to the gripper base (`panda_link8`) was not being created correctly by its macro.
-   **Lesson:** A URDF must have exactly one root. To fix this, we manually defined the link and joint instead of using the broken macro, forcing the two trees to connect.

## Future Work / Roadmap

-   [ ] **MoveIt 2 Integration:** Add a complete MoveIt configuration for the Panda arm to enable advanced motion planning, collision checking, and inverse kinematics.
-   [ ] **Path Planning:** Implement and test various path planning algorithms available in MoveIt and OMPL.
-   [ ] **Perception:** Integrate a simulated depth camera (such as the Intel RealSense) into the Gazebo environment.
-   [ ] **Object Detection:** Develop a ROS 2 node for detecting, localizing, and publishing the poses of objects in the camera's field of view.