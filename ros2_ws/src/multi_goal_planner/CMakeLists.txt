cmake_minimum_required(VERSION 3.8)
project(multi_goal_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python module
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/optimized_multi_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.py"
)

# Install map files
install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
  FILES_MATCHING
  PATTERN "*.json"
)

# Create __init__.py for Python package
install(CODE "
  file(WRITE \"\${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/${PROJECT_NAME}/__init__.py\" \"\")
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
