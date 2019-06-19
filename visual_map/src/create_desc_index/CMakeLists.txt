cmake_minimum_required(VERSION 2.8.3)
project(create_desc_index)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

cs_add_library(create_desc_index
    src/create_desc_index.cc
)

cs_add_executable(${PROJECT_NAME}_test
  src/main.cc
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

############
## EXPORT ##
############
cs_install()
cs_export()
