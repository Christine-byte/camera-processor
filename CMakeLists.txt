cmake_minimum_required(VERSION 3.5)
project(photo_capture)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(photo_capture_node src/photo_capture_node.cpp)
ament_target_dependencies(photo_capture_node rclcpp std_msgs sensor_msgs OpenCV cv_bridge image_transport)

install(TARGETS photo_capture_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
