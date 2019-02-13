cmake_minimum_required(VERSION 3.1)
project(test_match_global)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

message(STATUS ${GTEST_INCLUDE_DIRS})
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    rosbag
    libnabo
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/../visual_map/include
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GTEST_LIBRARIES}
    pthread
    /usr/local/lib/libprotobuf.a
)
