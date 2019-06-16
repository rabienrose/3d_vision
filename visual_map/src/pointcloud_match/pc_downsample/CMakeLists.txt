cmake_minimum_required(VERSION 2.8.3)
project(pc_downsample)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple()

add_definitions(-std=c++11 -DBENCHMARK_ENABLE)

cs_add_executable(pcdesc_matcher
  src/main.cpp
)

cs_install()
install(DIRECTORY configuration_files DESTINATION share/desc_matcher/)
cs_export()
