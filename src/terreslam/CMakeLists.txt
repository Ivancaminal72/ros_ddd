cmake_minimum_required(VERSION 2.8.3)
project(terreslam)
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
# find_package(OpenCV REQUIRED PATHS "~/local/opencv-3.2.0")
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
	${OpenCV_INCLUDE_DIRS}
	"~/local/opencv-3.2.0/include"
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

SET(Libraries
	${RTABMap_LIBRARIES}
	${OpenCV_LIBRARIES}
	"~/local/opencv-3.2.0/lib/libopencv_xfeatures2d.so"
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)

#############
# LIBRARIES #
#############

add_library(${PROJECT_NAME}
	src/comms/kitti_parser.cpp
	src/comms/kittiraw_common.cpp
	src/comms/kittiraw_ros_conversions.cpp
	src/features/plane_detector.cpp
	src/features/blob_detector.cpp
	src/features/dd_keypoint_detector.cpp
	## src/features/sunq_plane_extraction.cpp
	## src/features/ulysses_plane_extraction.cpp
	src/processings/plane_processor.cpp
	src/processings/blob_processor.cpp
	src/processings/dd_keypoint_processor.cpp
	src/registrations/dd_coarse_alignment.cpp
	## src/utils/ulysses_map.cpp
	## src/utils/ulysses_types.cpp
	## src/utils/sunq_map.cpp
	## src/utils/sunq_types.cpp
	src/utils/util_general.cpp
	src/utils/util_chrono.cpp
	src/utils/util_types.cpp
	src/utils/util_map.cpp
	src/utils/util_pcd.cpp
	src/frontend.cpp
	src/camera_model.cpp
	src/visualizer.cpp
	src/io_disk.cpp
)

# target_link_libraries(${PROJECT_NAME} ${Libraries})

add_library(${PROJECT_NAME}_plugins
	src/nodelets/rgb_depth_frontend.cpp
	src/nodelets/dd_keypoint_frontend.cpp
	src/nodelets/plane_detector_frontend.cpp
	src/nodelets/ddd_keypoint_frontend.cpp
	src/nodelets/blob_detector_frontend.cpp
	src/nodelets/metric_alignment_frontend.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME} ${Libraries})

############
# BINARIES #
############

add_executable(terreslam_rgb_depth_frontend src/nodelets/z_rgb_depth_frontend.cpp)
target_link_libraries(terreslam_rgb_depth_frontend ${Libraries})
set_target_properties(terreslam_rgb_depth_frontend PROPERTIES OUTPUT_NAME "rgb_depth_frontend")

add_executable(terreslam_dd_keypoint_frontend src/nodelets/z_dd_keypoint_frontend.cpp)
target_link_libraries(terreslam_dd_keypoint_frontend ${Libraries})
set_target_properties(terreslam_dd_keypoint_frontend PROPERTIES OUTPUT_NAME "dd_keypoint_frontend")

add_executable(terreslam_plane_detector_frontend src/nodelets/z_plane_detector_frontend.cpp)
target_link_libraries(terreslam_plane_detector_frontend ${Libraries})
set_target_properties(terreslam_plane_detector_frontend PROPERTIES OUTPUT_NAME "plane_detector_frontend")

add_executable(terreslam_ddd_keypoint_frontend src/nodelets/z_dd_keypoint_frontend.cpp)
target_link_libraries(terreslam_ddd_keypoint_frontend ${Libraries})
set_target_properties(terreslam_ddd_keypoint_frontend PROPERTIES OUTPUT_NAME "ddd_keypoint_frontend")

add_executable(terreslam_blob_detector_frontend src/nodelets/z_blob_detector_frontend.cpp)
target_link_libraries(terreslam_blob_detector_frontend ${Libraries})
set_target_properties(terreslam_blob_detector_frontend PROPERTIES OUTPUT_NAME "blob_detector_frontend")

add_executable(terreslam_metric_alignment_frontend src/nodelets/z_metric_alignment_frontend.cpp)
target_link_libraries(terreslam_metric_alignment_frontend ${Libraries})
set_target_properties(terreslam_metric_alignment_frontend PROPERTIES OUTPUT_NAME "metric_alignment_frontend")

###########
# INSTALL #
###########

install(TARGETS 
	terreslam_rgb_depth_frontend
	terreslam_plane_detector_frontend
	terreslam_blob_detector_frontend
	terreslam_metric_alignment_frontend
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
