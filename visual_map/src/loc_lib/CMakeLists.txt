cmake_minimum_required(VERSION 3.1)
project(loc_lib)
add_definitions(--std=c++11 -Wno-unknown-pragmas)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
cs_add_library(${PROJECT_NAME}
    src/ChamoLoc.cc
)

cs_install()
cs_export()

