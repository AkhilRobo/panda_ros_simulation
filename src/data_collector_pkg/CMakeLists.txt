cmake_minimum_required(VERSION 3.8)
project(data_collector_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Force C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(CollectDataForYolo src/DataCollect.cpp)
ament_target_dependencies(CollectDataForYolo rclcpp sensor_msgs cv_bridge OpenCV)
install(TARGETS CollectDataForYolo DESTINATION lib/${PROJECT_NAME})

ament_package()
