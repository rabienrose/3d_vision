cmake_minimum_required(VERSION 3.1)
project(visual_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

cs_add_library(${PROJECT_NAME} ${PROTO_SRCS}
    src/visual_map.cc
    src/frame.cc
    src/map_point.cc
    src/visual_map_seri.cc
)

cs_install()
cs_export()
