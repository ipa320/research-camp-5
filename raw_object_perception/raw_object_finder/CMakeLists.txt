cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


include_directories(
${PROJECT_SOURCE_DIR}/ros/include
)

rosbuild_add_library(raw_toolbox_ros ros/src/toolbox_ros.cpp)
rosbuild_add_library(raw_plane_extraction ros/src/plane_extraction.cpp)
rosbuild_add_library(raw_object_candidate_extraction ros/src/object_candidate_extraction.cpp)

rosbuild_add_executable(object_segmentation_node
	ros/src/nodes/object_segmentation_node.cpp
	ros/src/object_segmentation.cpp
	ros/src/roi_extraction.cpp
)
target_link_libraries(object_segmentation_node
	raw_toolbox_ros
	raw_plane_extraction
	raw_object_candidate_extraction
)

target_link_libraries(raw_object_candidate_extraction
	raw_plane_extraction
)
