cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alps_lib REQUIRED)

# CANとROSの中継ノード
add_executable(can_ros_bridge src/can_ros_bridge.cpp)
ament_target_dependencies(can_ros_bridge rclcpp alps_lib)

install(TARGETS
  can_ros_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
