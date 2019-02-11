cmake_minimum_required(VERSION 3.1)
project(test_visual_map)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    rosbag
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/../../pb
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cc
    ${PROJECT_SOURCE_DIR}/../../pb/VisualMap.pb.cc
    ${PROJECT_SOURCE_DIR}/../../pb/Frame.pb.cc
    ${PROJECT_SOURCE_DIR}/../../pb/MapPoint.pb.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    /usr/local/lib/libprotobuf.a
)
