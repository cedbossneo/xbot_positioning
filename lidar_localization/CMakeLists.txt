cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xbot_msgs
  slam_toolbox_msgs
  mower_map
  xbot_rpc
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_slam_bridge_node
  src/lidar_slam_bridge_node.cpp
  src/scan_filter.cpp
)
add_dependencies(lidar_slam_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_slam_bridge_node
  ${catkin_LIBRARIES}
)

add_executable(obstacle_tracker_node
  src/obstacle_tracker_node.cpp
  src/cluster_tracker.cpp
)
add_dependencies(obstacle_tracker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_tracker_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scan_filter test/test_scan_filter.cpp src/scan_filter.cpp)
  target_link_libraries(test_scan_filter ${catkin_LIBRARIES})

  catkin_add_gtest(test_cluster_tracker test/test_cluster_tracker.cpp src/cluster_tracker.cpp)
  target_link_libraries(test_cluster_tracker ${catkin_LIBRARIES})
endif()
