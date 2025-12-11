cmake_minimum_required(VERSION 3.8)
project(aos)

# CMake 정책 설정 (AUTOMOC 경고 제거)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lio_sam_wo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(OpenMP QUIET)

# Find GeographicLib library for GPS/UTM conversion
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GvdGraph.msg"
  "srv/GpsToRelative.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# aos_voxblox_interface library removed - file deleted

# tree_clusterer library removed - file deleted

# frontier_detector library removed - file deleted

# path_planner library removed - file deleted

# aos_anchor_generator library removed - file deleted

# aos_planner_node removed - file deleted
# add_executable(aos_planner_node src/aos_planner_node.cpp)
# ament_target_dependencies(aos_planner_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#   nav_msgs
#   visualization_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   pcl_ros
#   pcl_conversions
#   std_srvs
# )
# target_include_directories(aos_planner_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${EIGEN3_INCLUDE_DIRS}
# )

# Create the RViz panel plugin (must be SHARED for pluginlib)
# Find Qt5 for MOC processing
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Generate MOC files for Qt classes
qt5_wrap_cpp(MOC_FILES
  include/aos/aos_panel_plugin.hpp
)

add_library(aos_panel_plugin SHARED
  src/ui/aos_panel_plugin.cpp
  src/ui/aos_panel_plugin_ui.cpp
  src/ui/aos_panel_plugin_ros2.cpp
  src/ui/aos_panel_plugin_gps.cpp
  src/ui/aos_panel_plugin_fileio.cpp
  src/ui/aos_panel_plugin_params.cpp
  src/ui/aos_panel_plugin_remote.cpp
  ${MOC_FILES}
)





ament_target_dependencies(aos_panel_plugin
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_visual_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  lio_sam_wo
  rosidl_default_runtime
)

target_link_libraries(aos_panel_plugin
  Qt5::Core
  Qt5::Widgets
)

target_include_directories(aos_panel_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp
  ${EIGEN3_INCLUDE_DIRS}
)

# Add dependency on the generated message types
rosidl_get_typesupport_target(cpp_typesupport_target_panel "aos" "rosidl_typesupport_cpp")
target_link_libraries(aos_panel_plugin "${cpp_typesupport_target_panel}")

# Register RViz panel plugin description with pluginlib for base package rviz_common
pluginlib_export_plugin_description_file(rviz_common rviz/aos_panel_plugin.xml)

# Create the AOS GVD node
add_executable(aos_gvd_node 
  src/aos_gvd_node.cpp
  src/utils/voronoi_diagram.cpp
)
ament_target_dependencies(aos_gvd_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  Eigen3
  std_srvs
)
target_link_libraries(aos_gvd_node
  ${OpenCV_LIBS}
)
target_include_directories(aos_gvd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp
  ${EIGEN3_INCLUDE_DIRS}
)
# Add dependency on the generated message types
rosidl_get_typesupport_target(cpp_typesupport_target "aos" "rosidl_typesupport_cpp")
target_link_libraries(aos_gvd_node "${cpp_typesupport_target}")
if (OpenMP_CXX_FOUND)
  target_link_libraries(aos_gvd_node OpenMP::OpenMP_CXX)
endif()

# aos_tree_tracking_node removed - file deleted

# gvd_planning_node removed - file deleted
# add_executable(gvd_planning_node src/gvd_planning_node.cpp)
# ament_target_dependencies(gvd_planning_node
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   std_msgs
# )
# target_include_directories(gvd_planning_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
#   ${CMAKE_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp
# )
# rosidl_get_typesupport_target(cpp_typesupport_target "aos" "rosidl_typesupport_cpp")
# target_link_libraries(gvd_planning_node "${cpp_typesupport_target}")

# gvd_goal_planning_node removed - file deleted
# add_executable(gvd_goal_planning_node src/gvd_goal_planning_node.cpp)
# ament_target_dependencies(gvd_goal_planning_node
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   std_msgs
# )
# target_include_directories(gvd_goal_planning_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
#   ${CMAKE_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp
# )
# rosidl_get_typesupport_target(cpp_typesupport_target_goal "aos" "rosidl_typesupport_cpp")
# target_link_libraries(gvd_goal_planning_node "${cpp_typesupport_target_goal}")

# Create the AOS path generation node
add_executable(aos_path_gen_node src/aos_path_gen_node.cpp)
ament_target_dependencies(aos_path_gen_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  lio_sam_wo
)
target_include_directories(aos_path_gen_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp
)
# Add dependency on the generated message types
rosidl_get_typesupport_target(cpp_typesupport_target_cluster "aos" "rosidl_typesupport_cpp")
target_link_libraries(aos_path_gen_node "${cpp_typesupport_target_cluster}")

# Create the AOS path linearization node
add_executable(aos_path_linearization_node src/aos_path_linearization_node.cpp)
ament_target_dependencies(aos_path_linearization_node
  rclcpp
  nav_msgs
  geometry_msgs
)
target_include_directories(aos_path_linearization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Create the AOS state machine node
add_executable(aos_state_machine_node src/aos_state_machine_node.cpp)
ament_target_dependencies(aos_state_machine_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(aos_state_machine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# pointcloud_accumulator_node removed - file deleted
# occupancy_cluster_visualizer removed - file deleted

# Create the AOS seed generation node
add_executable(aos_seed_gen_node 
  src/aos_seed_gen_node.cpp
)
ament_target_dependencies(aos_seed_gen_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  OpenCV
  Eigen3
)
target_include_directories(aos_seed_gen_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(aos_seed_gen_node
  ${OpenCV_LIBS}
)

# wheel_odometry_node removed - file deleted

# Create the GPS to UTM conversion node
add_executable(gps_to_utm_node src/gps_to_utm_node.cpp)
ament_target_dependencies(gps_to_utm_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  rosidl_default_runtime
)
target_include_directories(gps_to_utm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)
target_link_libraries(gps_to_utm_node
  ${GEOGRAPHICLIB_LIBRARIES}
)
# Add nlohmann/json (header-only library)
find_package(nlohmann_json QUIET)
if(nlohmann_json_FOUND)
  target_link_libraries(gps_to_utm_node nlohmann_json::nlohmann_json)
else()
  # If not found via find_package, try to include it manually
  # nlohmann/json is header-only, so we can include it directly
  message(STATUS "nlohmann/json not found via find_package, using header-only include")
endif()
# Add dependency on the generated service types
rosidl_get_typesupport_target(cpp_typesupport_target_gps "aos" "rosidl_typesupport_cpp")
target_link_libraries(gps_to_utm_node "${cpp_typesupport_target_gps}")

# UTM-Odom Alignment, PCD Visualizer, Relative to UTM, and JSON Coordinate Converter nodes
# moved to tree_utm package

# Install executables
install(TARGETS
  # aos_planner_node  # removed - file deleted
  aos_gvd_node
  # aos_tree_tracking_node  # removed - file deleted
  # gvd_planning_node  # removed - file deleted
  # gvd_goal_planning_node  # removed - file deleted
  aos_path_gen_node
  aos_path_linearization_node
  aos_state_machine_node
  aos_seed_gen_node
  gps_to_utm_node
  aos_panel_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz panel plugin to standard lib path for pluginlib discovery
install(TARGETS aos_panel_plugin
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install RViz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install msg files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install srv files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
  rviz_common
  rviz_rendering
  rviz_visual_tools
  std_srvs
  rosidl_default_runtime
)

# Export include directories
ament_export_include_directories(include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests removed - test files deleted
  # ament_add_gtest(test_aos_planner test/test_aos_planner.cpp)
  
  # ESDF test executable removed - file deleted
  # add_executable(esdf_test_node src/esdf_test_node.cpp)
endif()

ament_package()