cmake_minimum_required(VERSION 3.8)
project(serial_communication_package)

# Standard CMake lines
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtle_msgs REQUIRED)

# Locate the libserialport
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# add_executable(serial_communication_node src/serial_communication_node.cpp)
# ament_target_dependencies(serial_communication_node rclcpp std_msgs)

# # Include and link directories for libserialport
# target_include_directories(serial_communication_node PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
# target_link_libraries(serial_communication_node ${LIBSERIALPORT_LIBRARIES})
add_executable(serial_communication_node src/serial_communication_node.cpp)
  # Dependencies by package name
ament_target_dependencies(serial_communication_node rclcpp turtle_msgs std_msgs)
target_include_directories(serial_communication_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(serial_communication_node PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS})
target_link_libraries(serial_communication_node ${LIBSERIALPORT_LIBRARIES})

# target_compile_options(${target} PRIVATE -Wall)
# Other Libraries
# Dependencies by lib name
# target_link_libraries(serial_communication_node ${PROJECT_NAME})



install(TARGETS serial_communication_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
