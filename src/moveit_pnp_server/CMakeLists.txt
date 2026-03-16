cmake_minimum_required(VERSION 3.8)
project(moveit_pnp_server)

# Default to C++17 (ROS2 standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_resources_panda_moveit_config)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MotionPlan.action"
  DEPENDENCIES geometry_msgs action_msgs
)

# Export action interface targets for use by executables in this package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare dependencies for executables
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  rviz_visual_tools
  interactive_markers
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# Example executable (uncomment and modify when implementing)
# add_executable(moveit_pnp_server_node src/moveit_pnp_server_node.cpp)
# ament_target_dependencies(moveit_pnp_server_node ${DEPENDENCIES})
# target_link_libraries(moveit_pnp_server_node "${cpp_typesupport_target}")

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes/scene files
install(DIRECTORY
  meshes
  scripts
  DESTINATION share/${PROJECT_NAME}
)
# Install Python scripts
install(PROGRAMS
  scripts/spawn_movable_box.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables (uncomment when you have executables)
# install(TARGETS
#   moveit_pnp_server_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules (uncomment if using Python)
# ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (uncomment if using Python scripts)
# install(PROGRAMS
#   scripts/my_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()