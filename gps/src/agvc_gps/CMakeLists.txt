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
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(AGVC_GPS_2D_NODE agvc_gps_2d_node)
set(AGVC_GPS_DIAGNOSTICS_NODE agvc_gps_diagnostics_node)
set(AGVC_GPS_ORIGIN_NODE agvc_gps_origin_node)

rosbuild_add_executable(${AGVC_GPS_2D_NODE}
  src/agvc_gps_2d_node.cpp
)

rosbuild_add_executable(${AGVC_GPS_DIAGNOSTICS_NODE}
  src/agvc_gps_diagnostics_node.cpp
)

rosbuild_add_executable(${AGVC_GPS_ORIGIN_NODE}
  src/agvc_gps_origin_node.cpp
)
