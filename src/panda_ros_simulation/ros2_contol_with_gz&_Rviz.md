graph LR
subgraph Launch_File
A[xacro panda.urdf.xacro] --> B[robot_description parameter]
B --> C[robot_state_publisher]
B --> D[spawn_entity.py]
end

subgraph Gazebo_World
D --> E[Gazebo Server]
E --> F[gazebo_ros2_control plugin]
end

subgraph Controller_Manager
F --> G[Controller Manager Node]
G --> H1[joint_state_broadcaster]
G --> H2[arm_controller]
G --> H3[eef_controller]
end

subgraph ROS2_Topics
I1[robot_description]
I2[joint_states]
I3[tf & tf_static]
I4[arm_controller/joint_trajectory]
I5[eef_controller/joint_trajectory]
end

subgraph Visualization
J[RViz]
K[MoveIt2 / User Node]
end

C --> I3
C --> I1
E --> F
F --> I2
H1 --> I2
I2 --> C
I2 --> J
I3 --> J
I1 --> J
I4 --> H2
I5 --> H3
K --> I4
K --> I5

%% Basic styles for GitHub
style Launch_File fill:#2d1f3d,color:#ffffff
style Gazebo_World fill:#1f2d3d,color:#ffffff
style Controller_Manager fill:#3d2d1f,color:#ffffff
style ROS2_Topics fill:#1f3d2d,color:#ffffff
style Visualization fill:#3d1f1f,color:#ffffff
style A fill:#ff00ff,color:#ffffff
style B fill:#3388ff,color:#ffffff
style C fill:#33ff77,color:#000000
style D fill:#33ff77,color:#000000
style E fill:#ff66ff,color:#000000
style F fill:#33ccff,color:#000000
style G fill:#ff00ff,color:#ffffff
style H1 fill:#33ff77,color:#000000
style H2 fill:#33ff77,color:#000000
style H3 fill:#33ff77,color:#000000
style I1 fill:#0099cc,color:#ffffff
style I2 fill:#0099cc,color:#ffffff
style I3 fill:#0099cc,color:#ffffff
style I4 fill:#0099cc,color:#ffffff
style I5 fill:#0099cc,color:#ffffff
style J fill:#ff4444,color:#ffffff
style K fill:#ffaa00,color:#000000
