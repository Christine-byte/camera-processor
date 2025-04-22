cmake_minimum_required(VERSION 3.5)
project(camera_processor)

# Find ROS 2 + dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 加入 include 路径，确保能找到 cv_bridge.h
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(camera_processor_node src/camera_processor_node.cpp)

# 链接依赖
ament_target_dependencies(camera_processor_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  camera_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
