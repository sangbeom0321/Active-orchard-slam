cmake_minimum_required(VERSION 3.8)
project(aos)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
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
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Create the orbit_voxblox_interface library
add_library(orbit_voxblox_interface
  src/orbit_voxblox_interface.cpp
)

ament_target_dependencies(orbit_voxblox_interface
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(orbit_voxblox_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the tree_clusterer library
add_library(tree_clusterer
  src/tree_clusterer.cpp
)

ament_target_dependencies(tree_clusterer
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(tree_clusterer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the frontier_detector library
add_library(frontier_detector
  src/frontier_detector.cpp
)

ament_target_dependencies(frontier_detector
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(frontier_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the path_planner library
add_library(path_planner
  src/path_planner.cpp
)

ament_target_dependencies(path_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(path_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the orbit_anchor_generator library
add_library(orbit_anchor_generator
  src/orbit_anchor_generator.cpp
)

ament_target_dependencies(orbit_anchor_generator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(orbit_anchor_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the orbit_planner_node executable
add_executable(orbit_planner_node
  src/orbit_planner_node.cpp
)

ament_target_dependencies(orbit_planner_node
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
  std_srvs
)

target_link_libraries(orbit_planner_node
  orbit_voxblox_interface
  tree_clusterer
  frontier_detector
  path_planner
  orbit_anchor_generator
  ${OpenCV_LIBS}
)

target_include_directories(orbit_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the RViz panel plugin (must be SHARED for pluginlib)
# Find Qt5 for MOC processing
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Generate MOC files for Qt classes
qt5_wrap_cpp(MOC_FILES
  include/aos/orbit_panel_plugin.hpp
)

add_library(orbit_panel_plugin SHARED
  src/orbit_panel_plugin.cpp
  ${MOC_FILES}
)





ament_target_dependencies(orbit_panel_plugin
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
)

target_link_libraries(orbit_panel_plugin
  Qt5::Core
  Qt5::Widgets
)

target_include_directories(orbit_panel_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Register RViz panel plugin description with pluginlib for base package rviz_common
pluginlib_export_plugin_description_file(rviz_common rviz/orbit_panel_plugin.xml)

# Create the GVD topology node
add_executable(gvd_topology_node src/gvd_topology_node.cpp)
ament_target_dependencies(gvd_topology_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)
target_include_directories(gvd_topology_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
  orbit_planner_node
  gvd_topology_node
  orbit_voxblox_interface
  tree_clusterer
  frontier_detector
  path_planner
  orbit_anchor_generator
  orbit_panel_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz panel plugin to standard lib path for pluginlib discovery
install(TARGETS orbit_panel_plugin
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
)

# Export targets
# ament_export_targets(has_targets)

# Export include directories
ament_export_include_directories(include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  ament_add_gtest(test_orbit_planner test/test_orbit_planner.cpp)
  target_link_libraries(test_orbit_planner
    orbit_voxblox_interface
    tree_clusterer
    frontier_detector
    path_planner
    orbit_anchor_generator
  )
  target_include_directories(test_orbit_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  # Add ESDF test executable
  add_executable(esdf_test_node src/esdf_test_node.cpp)
  ament_target_dependencies(esdf_test_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    pcl_conversions
  )
  target_link_libraries(esdf_test_node
    orbit_voxblox_interface
  )
  target_include_directories(esdf_test_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  # Install ESDF test executable
  install(TARGETS esdf_test_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()