cmake_minimum_required(VERSION 3.1)
project(visual_map)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_CXX_STANDARD 11)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    pb
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
    src/visual_map.cc
    src/frame.cc
    src/map_point.cc
    src/visual_map_seri.cc
    pb/VisualMap.pb.cc
    pb/Frame.pb.cc
    pb/MapPoint.pb.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
    /usr/local/lib/libprotobuf.a
)
