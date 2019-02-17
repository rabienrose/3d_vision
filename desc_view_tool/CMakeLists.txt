cmake_minimum_required(VERSION 3.1)
project(desc_view_tool)

set (CMAKE_CXX_STANDARD 11)
find_package(OpenCV 3 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
)

message(${OpenCV_INCLUDE_DIRS})
