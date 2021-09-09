cmake_minimum_required(VERSION 2.8.3)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(terreslam VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

find_package(catkin REQUIRED COMPONENTS 
						 roscpp rospy sensor_msgs std_msgs message_generation
						 cv_bridge image_transport minkindr minkindr_conversions
						 nodelet pluginlib
)
# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)
find_package(RTABMap REQUIRED)
# find_package(OpenCV REQUIRED PATHS "~/local/opencv_4.2.0")
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system program_options)
find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	 FILES
	 BlobMatches.msg
	 KeyPointMatches.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	 DEPENDENCIES
	 std_msgs
	 geometry_msgs
	 sensor_msgs
)

###########
## Build ##
###########

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${RTABMap_INCLUDE_DIRS}
	"~/local/opencv_4.2.0/include/opencv4"
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

SET(Libraries
	${RTABMap_LIBRARIES}
	"~/local/opencv_4.2.0/lib/libopencv_xfeatures2d.so"
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)

#############
# LIBRARIES #
#############

add_library(${PROJECT_NAME}
	src/features/plane_detector.cpp
	src/features/blob_detector.cpp
	src/features/dd_keypoint_detector.cpp
	src/features/ddd_keypoint_detector.cpp
	## src/features/sunq_plane_extraction.cpp
	## src/features/ulysses_plane_extraction.cpp
	src/processings/plane_processor.cpp
	src/processings/blob_processor.cpp
	src/processings/dd_keypoint_processor.cpp
	src/processings/ddd_keypoint_processor.cpp
	src/registrations/dd_coarse_alignment.cpp
	src/registrations/ddd_coarse_alignment.cpp
	## src/utils/ulysses_map.cpp
	## src/utils/ulysses_types.cpp
	## src/utils/sunq_map.cpp
	## src/utils/sunq_types.cpp
	src/utils/util_general.cpp
	src/utils/util_chrono.cpp
	src/utils/util_types.cpp
	src/utils/util_map.cpp
	src/utils/util_pcd.cpp
	src/nodelet.cpp
	src/camera_model.cpp
	src/visualizer.cpp
	src/io_disk.cpp
)

# target_link_libraries(${PROJECT_NAME} ${Libraries})

add_library(${PROJECT_NAME}_plugins
	src/nodelets/rgb_depth_nodelet.cpp
	src/nodelets/dd_keypoint_nodelet.cpp
	src/nodelets/plane_detector_nodelet.cpp
	src/nodelets/ddd_keypoint_nodelet.cpp
	src/nodelets/blob_detector_nodelet.cpp
	src/nodelets/metric_alignment_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME} ${Libraries})

############
# BINARIES #
############

add_executable(rgb_depth_nodelet src/nodelets/executables/rgb_depth_nodelet.cpp)
target_link_libraries(rgb_depth_nodelet ${Libraries})
set_target_properties(rgb_depth_nodelet PROPERTIES OUTPUT_NAME "rgb_depth_nodelet")

add_executable(dd_keypoint_nodelet src/nodelets/executables/dd_keypoint_nodelet.cpp)
target_link_libraries(dd_keypoint_nodelet ${Libraries})
set_target_properties(dd_keypoint_nodelet PROPERTIES OUTPUT_NAME "dd_keypoint_nodelet")

add_executable(plane_detector_nodelet src/nodelets/executables/plane_detector_nodelet.cpp)
target_link_libraries(plane_detector_nodelet ${Libraries})
set_target_properties(plane_detector_nodelet PROPERTIES OUTPUT_NAME "plane_detector_nodelet")

add_executable(ddd_keypoint_nodelet src/nodelets/executables/ddd_keypoint_nodelet.cpp)
target_link_libraries(ddd_keypoint_nodelet ${Libraries})
set_target_properties(ddd_keypoint_nodelet PROPERTIES OUTPUT_NAME "ddd_keypoint_nodelet")

add_executable(blob_detector_nodelet src/nodelets/executables/blob_detector_nodelet.cpp)
target_link_libraries(blob_detector_nodelet ${Libraries})
set_target_properties(blob_detector_nodelet PROPERTIES OUTPUT_NAME "blob_detector_nodelet")

add_executable(metric_alignment_nodelet src/nodelets/executables/metric_alignment_nodelet.cpp)
target_link_libraries(metric_alignment_nodelet ${Libraries})
set_target_properties(metric_alignment_nodelet PROPERTIES OUTPUT_NAME "metric_alignment_nodelet")

###########
# INSTALL #
###########

install(TARGETS 
	rgb_depth_nodelet
	dd_keypoint_nodelet
	plane_detector_nodelet
	ddd_keypoint_nodelet
	blob_detector_nodelet
	metric_alignment_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
	nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ##########
# # EXPORT #
# ##########
# cs_install()
# cs_export()
