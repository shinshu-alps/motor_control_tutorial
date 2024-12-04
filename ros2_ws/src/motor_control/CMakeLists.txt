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

# 視覚化ノード
add_executable(visualize src/visualize.cpp)
target_include_directories(visualize PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(visualize rclcpp tf2 tf2_geometry_msgs visualization_msgs alps_lib motor_control_interfaces)

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
ament_target_dependencies(sim_motor rclcpp alps_lib motor_control_interfaces)

# 制御指令送信ノード
add_executable(control_commander src/control_commander.cpp)
target_include_directories(control_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control_commander rclcpp alps_lib motor_control_interfaces)

# ノードのインストール
install(TARGETS
  visualize
  can_ros_bridge
  sim_motor
  control_commander
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
