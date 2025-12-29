cmake_minimum_required(VERSION 3.10)
project(sts_hardware_interface)

# Suppress FindBoost policy warning
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Find Python for SCServo_Linux bindings
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Build SCServo_Linux as a subdirectory
# This will compile the C++ library and Python bindings
add_subdirectory(external/SCServo_Linux)

# Make SCServo library available
set(SCSERVO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/SCServo_Linux/src)
set(SCSERVO_LIBRARY SCServo)

# STS Hardware Interface (ros2_control plugin)
add_library(${PROJECT_NAME} SHARED
  src/sts_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SCSERVO_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${SCSERVO_LIBRARY}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${diagnostic_updater_TARGETS}
)

# Export hardware interface as plugin
pluginlib_export_plugin_description_file(hardware_interface sts_hardware_interface.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES sts_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.xacro"
)

# Install documentation
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
  FILES_MATCHING PATTERN "*.md"
)

# Install SCServo library
install(
  TARGETS ${SCSERVO_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install SCServo headers
install(
  DIRECTORY ${SCSERVO_INCLUDE_DIR}/
  DESTINATION include/${PROJECT_NAME}/scservo
  FILES_MATCHING PATTERN "*.h"
)

# Install SCServo Python bindings to site-packages (optional)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/external/SCServo_Linux/scservo_python.cpython-312-aarch64-linux-gnu.so
  DESTINATION ${Python3_SITELIB}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  diagnostic_updater
)

ament_package()
