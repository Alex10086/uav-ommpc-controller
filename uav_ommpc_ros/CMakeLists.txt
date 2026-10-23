cmake_minimum_required(VERSION 3.16)
project(uav_ommpc_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uav_ommpc_interfaces REQUIRED)

add_executable(px4_bridge src/px4_bridge.cpp)
target_compile_features(px4_bridge PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(px4_bridge PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(px4_bridge rclcpp px4_msgs uav_ommpc_interfaces)
install(TARGETS px4_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
