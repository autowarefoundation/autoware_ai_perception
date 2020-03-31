cmake_minimum_required(VERSION 2.8.3)
project(vel_pose_diff_checker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_health_checker
  autoware_msgs
  autoware_system_msgs
  geometry_msgs
  roscpp
  roslint
  std_msgs
  tf
)

find_package(autoware_build_flags REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/vel_pose_diff_checker_node.cpp
  src/vel_pose_diff_checker_core.cpp
  src/value_time_queue.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()

  find_package(rostest REQUIRED)
  add_rostest_gtest(
    vel_pose_diff_checker-test
    test/test_vel_pose_diff_checker.test
    test/src/test_vel_pose_diff_checker.cpp
    src/vel_pose_diff_checker_core.cpp
    src/value_time_queue.cpp
  )
  target_link_libraries(vel_pose_diff_checker-test ${catkin_LIBRARIES})
endif()
