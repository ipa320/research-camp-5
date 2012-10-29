cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

rosbuild_find_ros_package(robot_srvs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


include_directories(
	${PROJECT_SOURCE_DIR}/ros/include
	${PROJECT_SOURCE_DIR}/common/include
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_library(${baseplacement} src/baseplacement.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# PLACEMENT WRT PLATFORM
rosbuild_add_executable(placement_wrt_workspace_service_server 
  ros/nodes/placement_wrt_workspace_service_server.cpp
  ros/src/placement_wrt_workspace/LaserScanLinearRegressionUtil.cpp
  common/src/placement_wrt_workspace/LaserScanLinearRegression.cpp)

rosbuild_add_executable(placement_wrt_workspace_action_server
  ros/nodes/placement_wrt_workspace_action_server.cpp
  ros/src/placement_wrt_workspace/LaserScanLinearRegressionUtil.cpp
  common/src/placement_wrt_workspace/LaserScanLinearRegression.cpp)

rosbuild_add_executable(placement_wrt_workspace_example
  ros/src/placement_wrt_workspace/test/basescan_orientation_example.cpp)
