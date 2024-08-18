cmake_minimum_required(VERSION 3.8)
project(dynamixel_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)


# include_directories(include)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)

# Build
add_executable(transform_node src/transform_node.cpp)
ament_target_dependencies(transform_node
  tf2
  tf2_ros
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
  rclcpp
)

# Install CPP Nodes
install(TARGETS
  transform_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install things for simulation
install(DIRECTORY
  meshes
  urdf
  launch
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)
# Python Nodes
install(PROGRAMS
  scripts/basic.py
  scripts/TransformHelpers.py
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
