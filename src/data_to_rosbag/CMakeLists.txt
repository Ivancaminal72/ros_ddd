cmake_minimum_required(VERSION 2.8.3)
project(data_to_rosbag)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/kittiraw_parser.cpp
  src/kittiraw_ros_conversions.cpp
  src/kittiraw_common.cpp

  src/kitti_parser.cpp
)

############
# BINARIES #
############
cs_add_executable(kittiraw_live_node src/kittiraw_live_node.cpp
)
target_link_libraries(kittiraw_live_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(kittiraw_rosbag_converter
  src/kittiraw_rosbag_converter.cpp
)
target_link_libraries(kittiraw_rosbag_converter ${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_add_executable(tiny_disp_view
  src/tiny_disp_view.cpp
)
target_link_libraries(tiny_disp_view ${PROJECT_NAME} ${OpenCV_LIBRARIES})





cs_add_executable(kitti_live_node
  src/kitti_live_node.cpp
)
target_link_libraries(kitti_live_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})


##########
# EXPORT #
##########
cs_install()
cs_export()