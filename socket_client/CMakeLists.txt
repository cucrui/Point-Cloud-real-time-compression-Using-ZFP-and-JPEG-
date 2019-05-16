cmake_minimum_required(VERSION 3.13)
project(my_socket_client)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ZFP REQUIRED)
include_directories(${ZFP_INCLUDE_DIRS})
link_directories(${ZFP_LIBRARY_DIRS})
add_definitions(${ZFP_DEFINITIONS})

set(CMAKE_PREFIX_PATH /opt/libjpeg-turbo)
find_package(PkgConfig REQUIRED)
pkg_search_module(TURBOJPEG REQUIRED libturbojpeg)

link_directories(${TURBOJPEG_LIBDIR})

add_executable(main main.cpp jpegcompression.h)
target_link_libraries (main ${PCL_LIBRARIES})
target_link_libraries (main ${ZFP_LIBRARIES})
target_link_libraries (main ${TurboJPEG_LIBRARIES})
target_include_directories(main PUBLIC ${TURBOJPEG_INCLUDE_DIRS})
target_link_libraries(main ${TURBOJPEG_LIBRARIES})