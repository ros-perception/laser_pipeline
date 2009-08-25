cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rospack(dense_laser_assembler)

rospack_add_library(dense_laser_assembler src/build_dense_laser_snapshot.cpp
                              					  src/dense_laser_msg_filter.cpp)
#rospack_add_executable(dense_laser_assembler_srv src/dense_laser_assembler_srv.cpp)
#target_link_libraries(dense_laser_assembler_srv dense_laser_assembler)

rospack_add_executable(joint_extractor_display src/joint_extractor_display.cpp)
#rospack_add_executable(tagged_laser_cache_display src/tagged_laser_cache_display.cpp)

rospack_add_executable(dense_laser_snapshotter src/dense_laser_snapshotter.cpp)
target_link_libraries(dense_laser_snapshotter dense_laser_assembler)

rospack_add_executable(laser_imager src/laser_imager.cpp)

#rospack_add_executable(talker src/talker.cpp)
#rospack_add_executable(listener src/listener.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

genmsg()
gensrv()
