cmake_minimum_required(VERSION 3.5)
project(camera_processor)

# 默认使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

# 添加可执行文件
add_executable(camera_processor_node src/camera_processor_node.cpp)

# 链接依赖库
ament_target_dependencies(camera_processor_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  nlohmann_json
)

# 安装目标
install(TARGETS
  camera_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
