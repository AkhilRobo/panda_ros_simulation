cmake_minimum_required(VERSION 3.8)
project(panda_ros_simulation)


find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)


install(
  DIRECTORY urdf meshes launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
