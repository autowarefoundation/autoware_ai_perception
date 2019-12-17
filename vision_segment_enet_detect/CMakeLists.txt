cmake_minimum_required(VERSION 2.8.12)
project(vision_segment_enet_detect)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)
find_package(CUDA)
find_package(OpenCV REQUIRED)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)


#####ENET########
##############################ENet's CAFFE FORK NEEDS TO BE PREVIOUSLY COMPILED####################
set(ENET_CAFFE_PATH "$ENV{HOME}/ENet/caffe-enet/distribute")
####################################################################################################
AW_CHECK_CUDA()

if(USE_CUDA AND EXISTS "${ENET_CAFFE_PATH}")
  add_executable(vision_segment_enet_detect
    nodes/vision_segment_enet_detect/vision_segment_enet_detect_node.cpp
    nodes/vision_segment_enet_detect/vision_segment_enet_detect.cpp
  )

  target_link_libraries(vision_segment_enet_detect
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${ENET_CAFFE_PATH}/lib/libcaffe.so
    glog
  )

  target_include_directories(vision_segment_enet_detect PRIVATE
    include
    ${CUDA_INCLUDE_DIRS}
    ${ENET_CAFFE_PATH}/include
  )

  add_dependencies(vision_segment_enet_detect
    ${catkin_EXPORTED_TARGETS}
  )

  install(
    TARGETS vision_segment_enet_detect
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
  )
else()
  message("' ENet/Caffe' is not installed. 'vision_segment_enet_detect' will not be built.")
endif()
