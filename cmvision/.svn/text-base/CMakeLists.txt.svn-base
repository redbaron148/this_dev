cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include( ${wxWidgets_USE_FILE} )
include_directories( ${wxWidgets_INCLUDE_DIRS} )

rosbuild_genmsg()

rosbuild_add_executable(cmvision
	src/cmvision.cc
	src/conversions.c
	src/cmvision_color_blob_finder.cpp
	src/main.cpp
)


rosbuild_add_executable(colorgui
	src/cmvision.cc
	src/conversions.c
	src/color_gui.cpp
)

target_link_libraries(colorgui fltk)
target_link_libraries(colorgui ${wxWidgets_LIBRARIES})
