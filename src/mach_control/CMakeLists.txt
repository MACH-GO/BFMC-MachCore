cmake_minimum_required(VERSION 3.8)
project(mach_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executables
add_executable(ackermann_controller src/ackermann_controller.cpp)
add_executable(ackermann_teleop_key src/ackermann_teleop_key.cpp)

# Include dirs
target_include_directories(ackermann_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(ackermann_teleop_key PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++ standard
target_compile_features(ackermann_controller PUBLIC cxx_std_17)
target_compile_features(ackermann_teleop_key PUBLIC cxx_std_17)

# Link dependencies
ament_target_dependencies(ackermann_controller
  rclcpp
  ackermann_msgs
  geometry_msgs
)
ament_target_dependencies(ackermann_teleop_key
  rclcpp
  ackermann_msgs
  geometry_msgs
)

# Install
install(TARGETS
  ackermann_controller
  ackermann_teleop_key
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
