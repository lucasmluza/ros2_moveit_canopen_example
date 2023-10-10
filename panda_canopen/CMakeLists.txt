cmake_minimum_required(VERSION 3.22)
project(panda_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)

cogen_dcf(cia402)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
