cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(alps_lib REQUIRED)
find_package(motor_control_interfaces REQUIRED)

# 現実のモーター（CANとROSの中継ノード）
add_executable(can_ros_bridge src/can_ros_bridge.cpp)
target_include_directories(can_ros_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_ros_bridge rclcpp alps_lib motor_control_interfaces)

# シミュレーション上のモーター
add_executable(sim_motor src/sim_motor.cpp)
target_include_directories(sim_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sim_motor rclcpp tf2 tf2_geometry_msgs visualization_msgs alps_lib motor_control_interfaces)

# 角度制御ノード
add_executable(angle_control src/angle_control.cpp)
target_include_directories(angle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(angle_control rclcpp alps_lib motor_control_interfaces)

# 速度制御ノード
add_executable(velocity_control src/velocity_control.cpp)
target_include_directories(velocity_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(velocity_control rclcpp alps_lib motor_control_interfaces)

# 角度-速度制御ノード
add_executable(angle_velocity_control src/angle_velocity_control.cpp)
target_include_directories(angle_velocity_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(angle_velocity_control rclcpp alps_lib motor_control_interfaces)

# ノードのインストール
install(TARGETS
  can_ros_bridge
  sim_motor
  angle_control
  velocity_control
  angle_velocity_control
  DESTINATION lib/${PROJECT_NAME})

# ディレクトリのインストール
install(DIRECTORY
  launch
  param_cfg
  rviz_cfg
  plotjuggler_cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
