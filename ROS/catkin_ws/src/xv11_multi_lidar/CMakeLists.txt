cmake_minimum_required(VERSION 2.8.3)
project(xv11_multi_lidar)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  aruco_mapping
  roscpp
  rospy
  roslaunch
  laser_geometry
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

roslaunch_add_file_check(launch)

###########
## Build ##
###########

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(xv11_raw2scan_wtf_node src/xv11_raw2scan_wtf_node.cpp)
add_executable(xv11_raw2imu_node src/xv11_raw2imu_node.cpp)
add_executable(scan2cloud_node src/scan2cloud_node.cpp)
add_executable(icp_node src/icp_node.cpp)
add_executable(cloud_accumulator_node src/cloud_accumulator_node.cpp)
add_executable(cloud2obstaclemap_node src/cloud2obstaclemap_node.cpp)
add_executable(aruco_pose_repack_node src/aruco_pose_repack_node.cpp)
add_executable(odom_to_map_tf_node src/odom_to_map_tf_node.cpp)
add_executable(cloud_coverage_metrics_node src/cloud_coverage_metrics_node.cpp)
add_executable(cloud_cluster_metrics_node src/cloud_cluster_metrics_node.cpp)
add_executable(var_scan_test_node src/var_scan_test_node.cpp)

add_dependencies(xv11_raw2scan_wtf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(xv11_raw2imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(scan2cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(icp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cloud_accumulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cloud2obstaclemap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_pose_repack_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_to_map_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cloud_coverage_metrics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cloud_cluster_metrics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(var_scan_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(xv11_raw2scan_wtf_node
  ${catkin_LIBRARIES}
)
target_link_libraries(xv11_raw2imu_node
  ${catkin_LIBRARIES}
)
target_link_libraries(scan2cloud_node
  ${catkin_LIBRARIES}
)
target_link_libraries(icp_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cloud_accumulator_node
  ${catkin_LIBRARIES}
)
target_link_libraries(odom_to_map_tf_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cloud2obstaclemap_node
  ${catkin_LIBRARIES}
)
target_link_libraries(aruco_pose_repack_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cloud_coverage_metrics_node
  ${catkin_LIBRARIES}
)
target_link_libraries(cloud_cluster_metrics_node
  ${catkin_LIBRARIES}
)
target_link_libraries(var_scan_test_node
  ${catkin_LIBRARIES}
)

# Install C++ and Python Executables
install(TARGETS 
  xv11_raw2scan_wtf_node 
  xv11_raw2imu_node 
  scan2cloud_node
  icp_node
  cloud_accumulator_node
  odom_to_map_tf_node
  cloud2obstaclemap_node
  aruco_pose_repack_node
  cloud_coverage_metrics_node
  cloud_cluster_metrics_node 
  var_scan_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS bin/adaptive_dither_dqn_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)