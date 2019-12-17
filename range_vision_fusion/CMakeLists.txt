cmake_minimum_required(VERSION 2.8.12)
project(range_vision_fusion)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  cv_bridge
  eigen_conversions
  image_geometry
  image_transport
  jsk_recognition_utils
  jsk_topic_tools
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)
find_package(OpenGL REQUIRED)

if (NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

catkin_package()

#fusion Library
add_library(range_vision_fusion_lib SHARED
  include/range_vision_fusion/range_vision_fusion.h
  src/range_vision_fusion.cpp
)

if(OPENMP_FOUND)
  set_target_properties(range_vision_fusion_lib PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

target_include_directories(range_vision_fusion_lib PRIVATE
  include
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(range_vision_fusion_lib
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(range_vision_fusion_lib
  ${catkin_EXPORTED_TARGETS}
)

#Fusion node
add_executable(range_vision_fusion
  src/range_vision_fusion_node.cpp
)
target_include_directories(range_vision_fusion PRIVATE
  include
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(range_vision_fusion
  range_vision_fusion_lib
)

install(
  TARGETS range_vision_fusion range_vision_fusion_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
