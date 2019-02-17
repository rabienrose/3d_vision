cmake_minimum_required(VERSION 3.1)
project(test_projected_desc)
set (CMAKE_CXX_STANDARD 11)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(test_projected_desc
  src/main.cc
)

cs_install()
cs_export()