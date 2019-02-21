cmake_minimum_required(VERSION 3.1)
project(test_visual_map)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(${PROJECT_NAME}
    src/main.cc
)

cs_add_library(${PROJECT_NAME}_lib src/project_desc.cc)

target_link_libraries(${PROJECT_NAME} 
    ${PROJECT_NAME}_lib
)

cs_install()
cs_export()
