cmake_minimum_required(VERSION 2.8.3)
project(lidar_point_pillars)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
find_package(autoware_build_flags REQUIRED)

AW_CHECK_CUDA()

if (USE_CUDA)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND OR INCOMPATIBLE CMAKE VERSION FOUND")
  set(CUDA_AVAIL OFF)
endif ()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSERS NAMES nvonnxparser)

if(NVINFER AND NVPARSERS AND NVONNXPARSERS)
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  message("NVONNXPARSERS: ${NVONNXPARSERS}")
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library."
)

if(CUDNN_LIBRARY)
  message("CUDNN is available!")
  message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Check TVM utility, runtime libraries and TVM files
find_library(TVM_LIBRARY NAMES tvm_runtime)
set(TVM_AVAIL OFF)

set(TVM_PFE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/tvm_models/tvm_point_pillars_pfe/)
set(TVM_RPN_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/tvm_models/tvm_point_pillars_rpn/)

if (EXISTS "${TVM_PFE_FOLDER}/deploy_graph.json"
    AND EXISTS "${TVM_PFE_FOLDER}/deploy_param.params"
    AND EXISTS "${TVM_PFE_FOLDER}/deploy_lib.so"
    AND EXISTS "${TVM_PFE_FOLDER}/inference_engine_tvm_config.hpp"
    AND EXISTS "${TVM_RPN_FOLDER}/deploy_graph.json"
    AND EXISTS "${TVM_RPN_FOLDER}/deploy_param.params"
    AND EXISTS "${TVM_RPN_FOLDER}/deploy_lib.so"
    AND EXISTS "${TVM_RPN_FOLDER}/inference_engine_tvm_config.hpp")
  set(TVM_AVAIL true)
endif()

if(CUDA_AVAIL AND ((TRT_AVAIL AND CUDNN_AVAIL) OR (TVM_AVAIL AND TVM_LIBRARY)))
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    pcl_ros
    autoware_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      roslib
      pcl_ros
      autoware_msgs
  )
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

  # Look for tvm_utility if this build is using TVM model
  if(TVM_AVAIL)
    find_package(catkin REQUIRED COMPONENTS
      tvm_vendor
      tvm_utility
    )
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTVM_IMPLEMENTATION")
    include_directories(tvm_models ${tvm_vendor_INCLUDE_DIRS}/tvm_vendor)
  endif()

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  set(SOURCE_FILES
    nodes/lidar_point_pillars_node.cpp
    nodes/point_pillars_ros.cpp
  )

  add_executable(lidar_point_pillars
    ${SOURCE_FILES}
  )

  add_dependencies(lidar_point_pillars
    ${catkin_EXPORTED_TARGETS}
  )


  cuda_add_library(gpu_point_pillars_lib
    nodes/preprocess_points_cuda.cu
    nodes/anchor_mask_cuda.cu
    nodes/scatter_cuda.cu
    nodes/postprocess_cuda.cu
    nodes/nms_cuda.cu
  )

  target_link_libraries(gpu_point_pillars_lib
    ${CUDA_LIBRARIES}
  )

  if(TVM_AVAIL)
    add_library(point_pillars_lib
      nodes/point_pillars.cpp
      nodes/preprocess_points.cpp
      nodes/pfe_tvm_pipeline.cpp
      nodes/rpn_tvm_pipeline.cpp
    )

    target_link_libraries(point_pillars_lib
      ${TVM_LIBRARY}
      ${CUDA_LIBRARIES}
      ${CUDA_CUBLAS_LIBRARIES}
      ${CUDA_curand_LIBRARY}
      gpu_point_pillars_lib
    )
  else()
    add_library(point_pillars_lib
      nodes/point_pillars.cpp
      nodes/preprocess_points.cpp
    )

    target_link_libraries(point_pillars_lib
      ${NVINFER}
      ${NVONNXPARSERS}
      ${CUDA_LIBRARIES}
      ${CUDA_CUBLAS_LIBRARIES}
      ${CUDA_curand_LIBRARY}
      ${CUDNN_LIBRARY}
      gpu_point_pillars_lib
    )
  endif()

  target_link_libraries(lidar_point_pillars
    ${catkin_LIBRARIES}
    point_pillars_lib
  )

  install(
    TARGETS
      gpu_point_pillars_lib
      point_pillars_lib
      lidar_point_pillars
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
  )

  install(DIRECTORY tvm_models/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tvm_models
  )

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
    PATTERN ".svn" EXCLUDE
  )

  if (CATKIN_ENABLE_TESTING AND EXISTS /proc/driver/nvidia/)
    find_package(rostest REQUIRED)
    catkin_add_gtest(test-point_pillars test/src/test_point_pillars.cpp)
    target_link_libraries(test-point_pillars ${catkin_LIBRARIES} point_pillars_lib)
  endif()
else()
  find_package(catkin REQUIRED)
  catkin_package()
  message("PointPillars won't be built, CUDA and/or TensorRT were not found.")
endif()
