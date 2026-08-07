cmake_minimum_required(VERSION 3.28)
project(sts_hardware_interface VERSION 0.3.0)

# Suppress FindBoost policy warning
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Require C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable clang-tidy if available
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks during compilation" OFF)
if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
  else()
    message(WARNING "clang-tidy not found. Install it to enable static analysis.")
  endif()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(control_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OneKeyCalibration.srv"
)

# Build SCServo_STS library (C++)
add_subdirectory(include/SCServo_STS)

# SCServo library configuration
set(SCSERVO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/SCServo_STS/include)
set(SCSERVO_LIBRARY SCServo)

# STS Hardware Interface (ros2_control plugin)
add_library(${PROJECT_NAME}_plugin SHARED
  src/sts_hardware_interface.cpp
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
  # Ensure the plugin can resolve rosidl-generated sibling libraries at runtime.
  # On macOS @loader_path expands to the directory containing the plugin dylib;
  # on Linux $ORIGIN does the same. Both the plugin and the typesupport libs are
  # installed into the same lib/ directory, so this single entry is sufficient.
  INSTALL_RPATH "$<IF:$<PLATFORM_ID:Darwin>,@loader_path,$ORIGIN>"
)
add_dependencies(${PROJECT_NAME}_plugin ${PROJECT_NAME}__rosidl_generator_cpp)

# Motor Diagnostics Node
add_executable(motor_diagnostics_node src/motor_diagnostics_node.cpp)
target_link_libraries(motor_diagnostics_node
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${control_msgs_TARGETS}
)
install(TARGETS motor_diagnostics_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SCSERVO_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${SCSERVO_LIBRARY}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin "${cpp_typesupport_target}")

# Export hardware interface as plugin
pluginlib_export_plugin_description_file(hardware_interface sts_hardware_interface.xml)

# Install library
install(TARGETS ${PROJECT_NAME}_plugin
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

# Install SCServo headers for downstream packages
install(
  DIRECTORY ${SCSERVO_INCLUDE_DIR}/scservo/
  DESTINATION include/${PROJECT_NAME}/scservo
  FILES_MATCHING PATTERN "*.h"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright and cpplint checks (optional linters)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Disable lint_cmake and uncrustify auto-detection: both linters would recurse into
  # include/SCServo_STS which is a vendor directory with its own style conventions.
  # Setting _FOUND before ament_lint_auto_find_test_dependencies() prevents hook
  # registration and avoids duplicate-test build errors.
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run automatic linters (lint_cmake and uncrustify are excluded above)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # C++ unit tests: pure conversion math (no ROS required)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # C++ unit tests: hardware interface lifecycle and mock behavior (needs rclcpp)
  ament_add_gtest(test_hardware_interface
    test/test_hardware_interface.cpp
  )
  target_link_libraries(test_hardware_interface
    ${PROJECT_NAME}_plugin
    ${rclcpp_TARGETS}
    hardware_interface::hardware_interface
    ${std_srvs_TARGETS}
  )
  target_include_directories(test_hardware_interface PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # Python launch tests: single motor velocity example with use_mock:=true
  add_launch_test(test/test_single_motor_velocity.launch.py)

  # Python launch tests: single motor position example with use_mock:=true
  add_launch_test(test/test_single_motor_position.launch.py)

  # Python launch tests: motor diagnostics integration
  add_launch_test(test/test_motor_diagnostics.launch.py)

  # Python launch tests: mixed mode example with use_mock:=true
  add_launch_test(test/test_mixed_mode.launch.py)
endif()

# Export package information for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_srvs
)

# Export targets for modern CMake usage
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# macOS + pixi/conda workaround: the rosidl typesupport dispatch does a bare
# dlopen("lib<pkg>__rosidl_typesupport_fastrtps_cpp.dylib") which only searches
# DYLD_LIBRARY_PATH and LC_RPATH entries of the calling process.  Hardened conda
# binaries strip DYLD_*, so the workspace install/lib dir is never searched.
# Fix: create symlinks in $CONDA_PREFIX/lib (always in the process rpath) that
# point back to the workspace-installed rosidl typesupport dylibs.
if(APPLE AND DEFINED ENV{CONDA_PREFIX})
  set(_pkg_name "${PROJECT_NAME}")  # expanded at configure time
  install(CODE "
    set(_conda_lib \"\$ENV{CONDA_PREFIX}/lib\")
    if(IS_DIRECTORY \"\${_conda_lib}\")
      file(GLOB _ts_libs
        \"\${CMAKE_INSTALL_PREFIX}/lib/lib${_pkg_name}__rosidl*.dylib\"
      )
      foreach(_src \${_ts_libs})
        get_filename_component(_name \"\${_src}\" NAME)
        set(_dst \"\${_conda_lib}/\${_name}\")
        message(STATUS \"Symlinking rosidl typesupport: \${_name}\")
        execute_process(
          COMMAND \${CMAKE_COMMAND} -E create_symlink \"\${_src}\" \"\${_dst}\"
        )
      endforeach()
    endif()
  ")
endif()

ament_package()
