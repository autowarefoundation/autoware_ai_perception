cmake_minimum_required(VERSION 2.8.3)
project(lidar_kf_contour_track)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  cv_bridge
  jsk_recognition_msgs
  op_planner
  op_ros_helpers
  pcl_conversions
  roscpp
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    op_ros_helpers
    cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#kl contour track
add_executable(lidar_kf_contour_track
  nodes/lidar_kf_contour_track/lidar_kf_contour_track.cpp
  nodes/lidar_kf_contour_track/lidar_kf_contour_track_core.cpp
  nodes/lidar_kf_contour_track/PolygonGenerator.cpp
  nodes/lidar_kf_contour_track/SimpleTracker.cpp
)
target_link_libraries(lidar_kf_contour_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(lidar_kf_contour_track
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    lidar_kf_contour_track
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
