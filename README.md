# Autonomous Robotic Arm: Pick, Place, and Maze Navigation

This project demonstrates the integration of perception, motion planning, and autonomous navigation using a simulated robotic arm in ROS 2 and Gazebo. The project is divided into two main phases:
1.  **Phase 1:** Autonomously detect a ball, plan a path to it, and pick it up.
2.  **Phase 2:** After picking up the ball, navigate through a maze to a goal location without collisions.

---

### Project Showcase

*(Once you have results, you can record short GIFs of your simulation and place them here.)*

| Phase 1: Pick and Place                                     | Phase 2: Maze Navigation                                        |
| ----------------------------------------------------------- | --------------------------------------------------------------- |
|![Panda Arm RViz Demo](media/Main.gif)|

---

## 🎯 Project Goals

- **Phase 1: Autonomous Pick and Place**
  - **Perception:** Use a simulated sensor (or a Gazebo state publisher) to detect and locate a target object (a ball) in 3D space.
  - **Motion Planning:** Utilize MoveIt to plan a collision-free trajectory from the arm's home position to the target object.
  - **Grasping:** Simulate the picking action by attaching the ball to the end-effector using a Gazebo plugin.

- **Phase 2: Maze Navigation**
  - **Environment Mapping:** Use a simulated camera on the arm to perceive the maze structure.
  - **Pathfinding:** Implement a pathfinding algorithm (e.g., A*) to determine the shortest sequence of waypoints for the end-effector to navigate from the start to the end of the maze.
  - **Constrained Motion Planning:** Use MoveIt to generate complex, full-arm trajectories that follow the waypoints while avoiding collisions with the maze walls, which are added to the MoveIt Planning Scene.

---

## 🛠️ Technology Stack

| Component            | Technology/Library                                   |
| -------------------- | ---------------------------------------------------- |
| **Framework** | ROS 2 (Humble)                                       |
| **Motion Planning** | MoveIt 2                                             |
| **Simulation** | Gazebo (Fortress)                                    |
| **Perception** | OpenCV, Gazebo Sensors (Camera)                      |
| **Core Language** | C++ (`rclcpp`)                                       |
| **Build System** | `colcon`                                             |

---

## 📈 Project Status

- **Phase 1: Pick and Place** - 🚧 In Progress
- **Phase 2: Maze Navigation** - 📅 Planned

---

## 🔧 Setup and Installation

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/AkhilRobo/panda_ros_simulation.git
    
    cd panda_ros_simulation
    ```

2.  **Install Dependencies**
    Ensure you have a working installation of ROS 2, MoveIt 2, and Gazebo.
    *(You can add specific dependency installation commands here later, e.g., `sudo apt install ros-humble-opencv-dev`)*

3.  **Build the Workspace**
    ```bash
    # From the root of your workspace
    colcon build --symlink-install
    ```

4.  **Source the Workspace**
    ```bash
    source install/setup.bash
    ```

---

## ▶️ How to Run

1.  **Launch the Phase 1 Demo (Pick and Place)**
    ```bash
    # This will launch Gazebo, RViz, MoveIt, and the application node
    ros2 launch your_package_name phase_one_demo.launch.py
    ```

2.  **Launch the Phase 2 Demo (Maze Navigation)**
    ```bash
    # This will launch the maze world and the maze navigation logic
    ros2 launch your_package_name phase_two_demo.launch.py
    ```

---

## 🗺️ Roadmap and Future Work

- [ ] Complete perception node for ball detection.
- [ ] Implement Gazebo plugin for grasping.
- [ ] Design and build the 3D maze model.
- [ ] Develop the maze-mapping and A* pathfinding node.
- [ ] Integrate pathfinding waypoints with MoveIt's motion planning.
- [ ] **(Stretch Goal)** Transition the project to physical hardware.