cmake_minimum_required(VERSION 2.8.3)
project(twist_generator)

add_compile_options(-std=c++11)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED
  amathutils_lib
  autoware_msgs
  geometry_msgs    
  roscpp
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES twist_generator
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_status_converter src/vehicle_status_converter.cpp src/vehicle_status_converter_node.cpp)
add_dependencies(vehicle_status_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_status_converter ${catkin_LIBRARIES})

install(
  TARGETS vehicle_status_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/twist_generator/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    vehicle_status_converter-test
    test/test_vehicle_status_converter.test 
    test/src/test_vehicle_status_converter.cpp
    src/vehicle_status_converter.cpp
  )
  target_link_libraries(vehicle_status_converter-test ${catkin_LIBRARIES})
endif()
