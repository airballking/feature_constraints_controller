cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories($(EIGEN_INCLUDE_DIRS))
add_definitions($(EIGEN_DEFINITIONS))

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fivopts -funsafe-math-optimizations -g") # allow more inlining

file(MAKE_DIRECTORY bin/test)

rosbuild_add_library(${PROJECT_NAME} src/FeatureConstraints.cpp src/Controller.cpp src/Conversions.cpp src/Analysis.cpp src/ChainRPY.cpp src/JointLimitAvoidanceController.cpp)

rosbuild_add_gtest(test/test_FeatureConstraints src/test_FeatureConstraints.cpp src/FeatureConstraints.cpp src/ChainRPY.cpp)
rosbuild_add_gtest(test/test_Analysis src/test_Analysis.cpp src/Analysis.cpp src/FeatureConstraints.cpp src/ChainRPY.cpp)
rosbuild_add_rostest(test/constraints_rank.test)

rosbuild_add_executable(analysis src/FeatureConstraints.cpp src/Analysis_main.cpp src/Analysis.cpp src/ChainRPY.cpp)
rosbuild_add_executable(rank_test_service src/rank_test_service.cpp src/Analysis.cpp src/Conversions.cpp src/FeatureConstraints.cpp src/ChainRPY.cpp src/Controller.cpp)
