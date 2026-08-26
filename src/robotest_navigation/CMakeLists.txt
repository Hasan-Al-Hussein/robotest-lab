# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(robotest_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees config launch maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/lifecycle_startup_trigger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lifecycle_startup_trigger
)

install(
  PROGRAMS tools/generate_map.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_robotest_navigation_contract
    test/test_navigation_contract.py
    TIMEOUT 90
  )
  ament_add_pytest_test(
    test_lifecycle_startup_trigger
    test/test_lifecycle_startup_trigger.py
    TIMEOUT 60
  )
endif()

ament_package()
