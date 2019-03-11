cmake_minimum_required(VERSION 2.8)
project(vocabulary_tree)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
add_definitions(-std=c++11 -mssse3 )

cs_add_executable(${PROJECT_NAME}
               src/helpers.cc
               src/vocabulary-tree-maker.cc)

cs_install()
cs_export()
