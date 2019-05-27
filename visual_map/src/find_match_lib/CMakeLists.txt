cmake_minimum_required(VERSION 3.1)
project(find_match_lib)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
cs_add_library(${PROJECT_NAME}
    src/FindMatchBetweenFrames.cc
)

cs_add_executable(${PROJECT_NAME}_test
    src/main.cc
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
cs_add_executable(${PROJECT_NAME}_test_img
    src/main_img.cc
)
target_link_libraries(${PROJECT_NAME}_test_img ${PROJECT_NAME})

cs_install()
cs_export()

