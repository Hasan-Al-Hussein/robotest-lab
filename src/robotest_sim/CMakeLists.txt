# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.12)
project(robotest_sim)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(robotest_contact_stream_policy STATIC
  src/contact_stream_gate.cpp
)
target_compile_features(robotest_contact_stream_policy PUBLIC cxx_std_17)
target_compile_options(robotest_contact_stream_policy PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(robotest_contact_stream_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robotest_contact_stream_policy
  builtin_interfaces
  geometry_msgs
  ros_gz_interfaces
  std_msgs
)

add_executable(contact_stream_gate src/contact_stream_gate_node.cpp)
target_compile_features(contact_stream_gate PUBLIC cxx_std_17)
target_compile_options(contact_stream_gate PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(contact_stream_gate robotest_contact_stream_policy)
ament_target_dependencies(contact_stream_gate rclcpp ros_gz_interfaces rosgraph_msgs)

# Embed the same canonical source-inventory digest that the coverage-manifest
# generator derives. Runtime/release gates can therefore reject a stale ELF
# even when its executable name and graph endpoints look correct.
set(_contact_gate_repository_paths
  "src/robotest_sim/CMakeLists.txt"
  "src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp"
  "src/robotest_sim/src/contact_stream_gate.cpp"
  "src/robotest_sim/src/contact_stream_gate_node.cpp"
)
set(_contact_gate_package_paths
  "CMakeLists.txt"
  "include/robotest_sim/contact_stream_gate.hpp"
  "src/contact_stream_gate.cpp"
  "src/contact_stream_gate_node.cpp"
)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robotest_sim/contact_stream_gate.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/contact_stream_gate.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/contact_stream_gate_node.cpp"
)
set(_contact_gate_inventory_json "{\"schema_version\":1,\"sources\":[")
set(_contact_gate_separator "")
foreach(_index RANGE 0 3)
  list(GET _contact_gate_repository_paths ${_index} _repository_path)
  list(GET _contact_gate_package_paths ${_index} _package_path)
  file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${_package_path}" _source_sha256)
  string(APPEND _contact_gate_inventory_json
    "${_contact_gate_separator}{\"path\":\"${_repository_path}\","
    "\"sha256\":\"${_source_sha256}\"}"
  )
  set(_contact_gate_separator ",")
endforeach()
string(APPEND _contact_gate_inventory_json "]}\n")
string(SHA256 ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256
  "${_contact_gate_inventory_json}"
)
string(CONCAT _contact_gate_compile_definition
  "ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256=\""
  "${ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256}"
  "\""
)
target_compile_definitions(contact_stream_gate PRIVATE
  "${_contact_gate_compile_definition}"
)

install(
  TARGETS contact_stream_gate robotest_contact_stream_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/phase1_runtime_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_contact_stream_gate test/test_contact_stream_gate.cpp)
  target_link_libraries(test_contact_stream_gate robotest_contact_stream_policy)
  ament_add_pytest_test(
    test_robotest_sim_structure
    test/test_structure.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_contact_stream_gate_build
    test/test_contact_stream_gate_build.py
    TIMEOUT 960
  )
endif()

ament_export_dependencies(builtin_interfaces geometry_msgs ros_gz_interfaces std_msgs)
ament_export_include_directories(include)
ament_package()
