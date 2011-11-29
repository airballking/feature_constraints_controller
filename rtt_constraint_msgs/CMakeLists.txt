cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_include(rtt_rosnode GenerateRTTtypekit)
ros_generate_rtt_typekit(constraint_msgs)

