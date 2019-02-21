cmake_minimum_required(VERSION 3.1)
project(descriptor_projection)
set (CMAKE_CXX_STANDARD 11)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(LIBRARY_NAME ${PROJECT_NAME})

cs_add_library(${LIBRARY_NAME} src/build-projection-matrix.cc
                               src/flags.cc
                               )

cs_install()
cs_export()
