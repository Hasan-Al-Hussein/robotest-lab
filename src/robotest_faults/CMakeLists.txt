# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(robotest_faults)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotest_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(robotest_faults_core SHARED
  src/fault_protocol.cpp
  src/pass_through.cpp
)
target_include_directories(robotest_faults_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(robotest_faults_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(robotest_faults_core
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(robotest_faults_core
  builtin_interfaces
  geometry_msgs
  nav_msgs
  robotest_interfaces
  sensor_msgs
  tf2
  tf2_geometry_msgs
  unique_identifier_msgs
)

add_executable(fault_proxy_node
  src/fault_proxy_node.cpp
  src/main.cpp
)
target_compile_options(fault_proxy_node PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(fault_proxy_node robotest_faults_core)
ament_target_dependencies(fault_proxy_node
  geometry_msgs
  nav_msgs
  rclcpp
  robotest_interfaces
  sensor_msgs
  std_srvs
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS robotest_faults_core fault_proxy_node
  EXPORT export_robotest_faults
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_pass_through test/test_pass_through.cpp)
  target_link_libraries(test_pass_through robotest_faults_core)
  ament_target_dependencies(test_pass_through rclcpp)

  ament_add_gtest(test_fault_protocol test/test_fault_protocol.cpp)
  target_link_libraries(test_fault_protocol robotest_faults_core)

  ament_add_gtest(test_fault_application test/test_fault_application.cpp)
  target_link_libraries(test_fault_application robotest_faults_core)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_robotest_faults HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  robotest_interfaces
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  unique_identifier_msgs
)
ament_package()
