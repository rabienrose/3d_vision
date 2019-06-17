cmake_minimum_required(VERSION 3.1)
project(loc_lib)
add_definitions(--std=c++11 -Wno-unknown-pragmas -w)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

file(GLOB ALL_HEADER_FILES "include/*")

file(COPY ${ALL_HEADER_FILES} DESTINATION ${CATKIN_DEVEL_PREFIX}/include)

cs_add_library(${PROJECT_NAME}
    src/ChamoLoc.cc
    src/FeatureDistance.cpp
    src/FeatureCoordinates.cpp
    src/Camera.cpp
    src/CameraCalibration.cpp
)

cs_add_executable(${PROJECT_NAME}_test
    src/main.cc
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()

