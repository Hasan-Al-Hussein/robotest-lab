# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(robotest_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS tools/phase1_runtime_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_robotest_sim_structure
    test/test_structure.py
    TIMEOUT 60
  )
endif()

ament_package()
