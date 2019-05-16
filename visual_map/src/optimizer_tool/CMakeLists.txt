cmake_minimum_required(VERSION 3.1)
project(optimizer_tool)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)

cs_add_library(${PROJECT_NAME} 
    src/optimizer_tool.cc
    #src/opti_imu.cc
)

cs_add_executable(${PROJECT_NAME}_test
    src/main.cc
    src/visual_tool.cc
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
