cmake_minimum_required(VERSION 3.8)
project(sts_hardware_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Find SCServo library from lekiwi_ros2 package
find_package(lekiwi_ros2 REQUIRED)

# Get SCServo include directory and library from lekiwi_ros2
set(SCSERVO_INCLUDE_DIR ${lekiwi_ros2_DIR}/../../../src/lekiwi_ros2/external/SCServo_Linux/src)

# STS Hardware Interface (ros2_control plugin)
add_library(${PROJECT_NAME} SHARED
  src/sts_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${SCSERVO_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  SCServo
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
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
  lekiwi_ros2
)

ament_package()
