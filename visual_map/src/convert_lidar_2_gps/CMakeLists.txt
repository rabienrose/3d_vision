cmake_minimum_required(VERSION 3.1)
project(convert_lidar_2_gps)


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)
cs_add_executable(${PROJECT_NAME}
    src/main.cc
)

cs_install()
cs_export()

