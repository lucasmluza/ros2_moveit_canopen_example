cmake_minimum_required(VERSION 3.22)
project(panda_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

cogen_dcf(cia402)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

add_executable(canopen_joint_state_publisher src/canopen_joint_state_publisher.cpp)
ament_target_dependencies(canopen_joint_state_publisher rclcpp sensor_msgs urdf kdl_parser)

install(
  TARGETS canopen_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
