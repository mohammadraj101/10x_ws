# cmake_minimum_required(VERSION 3.8)
# project(dwa_planner)

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

# # Include directories
# include_directories(
#   include
# )

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GetGoal.srv"
# )

# ament_export_dependencies(rosidl_default_runtime)


# # Add executable
# add_executable(dwa_planner_node src/dwa_planner_node.cpp)

# # Link libraries
# ament_target_dependencies(dwa_planner_node
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# # rosidl_target_interfaces(dwa_planner_node
# #   ${PROJECT_NAME} rosidl_typesupport_cpp)
# # target_link_libraries(dwa_planner_node ${cpp_typesupport_target})

# # Install target
# install(TARGETS
#   dwa_planner_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )



# # Export package dependencies
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(nav_msgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(visualization_msgs)
# ament_export_dependencies(tf2)
# ament_export_dependencies(tf2_ros)
# ament_export_dependencies(tf2_geometry_msgs)






# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(dwa_planner_node ${cpp_typesupport_target})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # ament package
# ament_package()



cmake_minimum_required(VERSION 3.8)
project(dwa_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  

# Generate service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGoal.srv"
  DEPENDENCIES std_msgs
)

# Ensure dependencies are built in the correct order
ament_export_dependencies(rosidl_default_runtime)

# Add Executables
add_executable(dwa_planner_node src/dwa_planner_node.cpp)
add_executable(working_dwa src/working_backup_my_world.cpp)
add_executable(check_dwa src/checker.cpp)
add_executable(new_dwa src/new_dwa.cpp)
add_executable(local_costmap_node src/local_costmap.cpp)
# Link dependencies

ament_target_dependencies(local_costmap_node rclcpp sensor_msgs nav_msgs tf2_geometry_msgs)

ament_target_dependencies(dwa_planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(working_dwa
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(check_dwa 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(new_dwa 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link the generated service header to the node
rosidl_target_interfaces(dwa_planner_node
  ${PROJECT_NAME} rosidl_typesupport_cpp)

rosidl_target_interfaces(working_dwa
  ${PROJECT_NAME} rosidl_typesupport_cpp)

rosidl_target_interfaces(check_dwa
  ${PROJECT_NAME} rosidl_typesupport_cpp)

rosidl_target_interfaces(new_dwa
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Install target
install(TARGETS
  dwa_planner_node
  local_costmap_node
  new_dwa
  working_dwa
  check_dwa
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package dependencies
ament_package()

