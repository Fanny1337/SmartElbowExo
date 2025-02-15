cmake_minimum_required(VERSION 3.8)
project(control_vr_package)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})

# Python script installation
install(PROGRAMS
  scripts/trajectory_planner.py
  RENAME trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Python script installation
install(PROGRAMS
  scripts/plot_python.py
  RENAME plot_python
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# C++ executable for control node
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs Eigen3)

# Install C++ executable
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# C++ executable for dynamic model node
add_executable(dynamic_model src/virtual_robot_node.cpp)
ament_target_dependencies(dynamic_model rclcpp std_msgs Eigen3)

# Install C++ executable
install(TARGETS
  dynamic_model
  DESTINATION lib/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package setup
ament_package()
