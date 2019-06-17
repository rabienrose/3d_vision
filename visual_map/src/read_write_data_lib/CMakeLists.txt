cmake_minimum_required(VERSION 2.8)
project(read_write_data_lib)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

file(GLOB ALL_HEADER_FILES "include/*")

file(COPY ${ALL_HEADER_FILES} DESTINATION ${CATKIN_DEVEL_PREFIX}/include)

cs_add_library(${PROJECT_NAME} SHARED
src/read_write.cpp
)

cs_install()
cs_export()
