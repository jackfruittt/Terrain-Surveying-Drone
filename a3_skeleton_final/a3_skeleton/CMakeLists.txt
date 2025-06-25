cmake_minimum_required(VERSION 3.5)
project(a3_skeleton)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

## Find packages, remove those that are not needed
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_library(common_lib src/common.cpp)
ament_target_dependencies(common_lib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  visualization_msgs
  grid_map_ros
  grid_map_msgs
)

#################################################################
# The library we are developing will be called laserprocessing
# Laser processing library
add_library(laserprocessing src/laserprocessing.cpp)
ament_target_dependencies(laserprocessing rclcpp std_msgs sensor_msgs nav_msgs)
target_link_libraries(laserprocessing common_lib)

# Graph library
add_library(graph src/graph.cpp)
ament_target_dependencies(graph rclcpp)
target_link_libraries(graph common_lib)

# Controller library
add_library(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp)
target_link_libraries(controller common_lib)

# Waypoint manager library
add_library(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager rclcpp)
target_link_libraries(waypoint_manager common_lib)

# Add terrain_map library BEFORE sensor_node_lib
add_library(terrain_map src/terrain_map.cpp)
ament_target_dependencies(terrain_map
  rclcpp
  grid_map_msgs
  grid_map_ros
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(terrain_map common_lib)

# TSP solver library
add_library(tsp_solver src/tsp_solver.cpp)
ament_target_dependencies(tsp_solver 
  rclcpp
  geometry_msgs
  visualization_msgs
)
target_link_libraries(tsp_solver common_lib)

# Sensor node library
add_library(sensor_node_lib src/sensor_node.cpp)
ament_target_dependencies(sensor_node_lib
  rclcpp
  sensor_msgs
  nav_msgs
  grid_map_msgs
  grid_map_ros
)
target_link_libraries(sensor_node_lib common_lib laserprocessing terrain_map)

# Add the drone_node executable
add_executable(drone_node src/drone_main.cpp src/drone_node.cpp)
ament_target_dependencies(drone_node 
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  grid_map_msgs
  grid_map_ros
  std_srvs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(drone_node 
  common_lib 
  laserprocessing 
  controller 
  waypoint_manager 
  sensor_node_lib
  terrain_map
  graph
  tsp_solver
)

# Add the sensor node executable
add_executable(sensor_node src/sensor_main.cpp)
ament_target_dependencies(sensor_node
  rclcpp
  sensor_msgs
  nav_msgs
  grid_map_msgs
  grid_map_ros
)
target_link_libraries(sensor_node common_lib sensor_node_lib)

install(TARGETS
  common_lib
  drone_node
  sensor_node
  sensor_node_lib
  laserprocessing
  controller
  waypoint_manager
  terrain_map
  graph
  tsp_solver
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    data
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts directory
install(
  DIRECTORY
    scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# ========================================
# TESTING SECTION (Cleaned up for single comprehensive test)
# ========================================
if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_lint_auto_find_test_dependencies()

  # Single comprehensive test for all assignment requirements
  ament_add_ros_isolated_gtest(drone_survey_tests test/drone_survey_tests.cpp)
  target_link_libraries(drone_survey_tests
    # All your libraries needed for testing
    common_lib
    terrain_map
    controller
    waypoint_manager
    tsp_solver
    graph
    laserprocessing
    sensor_node_lib
    ament_index_cpp::ament_index_cpp
  )
  ament_target_dependencies(drone_survey_tests
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    grid_map_ros
    grid_map_msgs
    visualization_msgs
    rosbag2_cpp
    rosbag2_storage
    tf2
    tf2_geometry_msgs
  )

  # Install test data directory
  install(DIRECTORY data
    DESTINATION share/${PROJECT_NAME}/test
    PATTERN "*"
    OPTIONAL
  )

  # Install test scripts
  install(PROGRAMS
    scripts/collect_test_data.sh
    DESTINATION lib/${PROJECT_NAME}
    OPTIONAL
  )

endif()

ament_package()
