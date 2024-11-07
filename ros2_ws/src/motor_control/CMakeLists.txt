cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alps_lib REQUIRED)
find_package(motor_control_interfaces REQUIRED)

# CANとROSの中継ノード
add_executable(can_ros_bridge src/can_ros_bridge.cpp)
ament_target_dependencies(can_ros_bridge rclcpp alps_lib motor_control_interfaces)

# 角度制御ノード
add_executable(angle_control src/angle_control.cpp)
target_include_directories(angle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(angle_control rclcpp alps_lib motor_control_interfaces)

install(TARGETS
  can_ros_bridge
  angle_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
