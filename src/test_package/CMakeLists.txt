cmake_minimum_required(VERSION 3.5)
project(test_package)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(test_torque_publisher src/test_torque_publisher.cpp)


# Specify libraries to link a library or executable target against
ament_target_dependencies(test_torque_publisher rclcpp std_msgs)

# Install the executable
install(TARGETS
  test_torque_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_package()
