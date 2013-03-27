cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(./src/)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

FIND_PACKAGE(Qt4 REQUIRED)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

SET(navigationISL_HEADERS src/rosThread.h)

SET(navigationISL_SOURCES src/rosThread.cpp)

QT4_WRAP_CPP(navigationISL_HEADERS_MOC ${navigationISL_HEADERS})

qt4_automoc(${navigationISL_SOURCES})

include_directories(./src/)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} mpfr ${QT_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(navigation src/main.cpp src/navigationController.cpp src/navigationController.h ${navigationISL_SOURCES} ${navigationISL_HEADERS_MOC})
target_link_libraries(navigation ${PROJECT_NAME})
