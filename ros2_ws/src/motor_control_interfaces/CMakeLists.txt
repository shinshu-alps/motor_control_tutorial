cmake_minimum_required(VERSION 3.8)
project(motor_control_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(alps_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlTarget.msg"
  DEPENDENCIES alps_interfaces
)

ament_package()
