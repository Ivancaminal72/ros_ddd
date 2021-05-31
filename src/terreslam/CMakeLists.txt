cmake_minimum_required(VERSION 2.8.3)
project(terreslam)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system program_options)
find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/comms/kitti_parser.cpp
  src/comms/kittiraw_common.cpp
  src/comms/kittiraw_ros_conversions.cpp
  src/features/plane_detector.cpp
  src/features/blob_detector.cpp
  src/features/keypoint_detector.cpp
  ## src/features/sunq_plane_extraction.cpp
  ## src/features/ulysses_plane_extraction.cpp
  src/processings/plane_processor.cpp
  src/processings/blob_processor.cpp
  src/processings/keypoint_processor.cpp
  ## src/utils/ulysses_map.cpp
  ## src/utils/ulysses_types.cpp
  ## src/utils/sunq_map.cpp
  ## src/utils/sunq_types.cpp
  src/utils/util_msg.cpp
  src/utils/util_algebra.cpp
  src/utils/util_types.cpp
  src/utils/util_map.cpp
  src/utils/util_pcd.cpp
  src/frontend.cpp
  src/camera_model.cpp
  src/visualizer.cpp
  src/io_disk.cpp
)

# target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

cs_add_library(${PROJECT_NAME}_plugins
  src/nodelets/rgb_depth_frontend.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME} ${PCL_LIBRARIES})

############
# BINARIES #
############

cs_add_executable(terreslam_rgb_depth_frontend src/nodelets/z_rgb_depth_frontend.cpp)
target_link_libraries(terreslam_rgb_depth_frontend ${PCL_LIBRARIES})
set_target_properties(terreslam_rgb_depth_frontend PROPERTIES OUTPUT_NAME "rgb_depth_frontend")


##########
# EXPORT #
##########
cs_install()
cs_export()
