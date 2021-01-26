cmake_minimum_required(VERSION 2.8.3)
project(terreslam)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system program_options)
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

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

#############
# LIBRARIES #
#############

cs_add_library(${PROJECT_NAME}
  src/comms/kitti_parser.cpp
  src/comms/kittiraw_common.cpp
  src/comms/kittiraw_ros_conversions.cpp
  src/features/plane_detector.cpp
  src/utils/util_msg.cpp
  src/utils/util_algebra.cpp
  src/frontend.cpp
  src/camera_model.cpp
)

cs_add_library(${PROJECT_NAME}_plugins
  src/nodelets/rgb_depth_frontend.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME})

############
# BINARIES #
############

cs_add_executable(terreslam_rgb_depth_frontend src/nodelets/z_rgb_depth_frontend.cpp)
target_link_libraries(terreslam_rgb_depth_frontend)
set_target_properties(terreslam_rgb_depth_frontend PROPERTIES OUTPUT_NAME "rgb_depth_frontend")


##########
# EXPORT #
##########
cs_install()
cs_export()
