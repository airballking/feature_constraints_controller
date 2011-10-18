CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

project(itasc)

set (ROS_ROOT $ENV{ROS_ROOT} )


include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo) #set type to Debug to get debug info compiled
rosbuild_init()


rosbuild_find_ros_package(rtt )
rosbuild_find_ros_package(ocl )

set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
set( OCL_HINTS HINTS ${ocl_PACKAGE_PATH}/install )


find_package(OROCOS-RTT HINTS ${rtt_PACKAGE_PATH}/install )
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)


list(APPEND CMAKE_MODULE_PATH ${ocl_PACKAGE_PATH}/config)

find_package(RTTPlugin REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.

add_subdirectory(src)
