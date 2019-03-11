cmake_minimum_required(VERSION 3.1)
project(train_projected_desc)
set (CMAKE_CXX_STANDARD 11)
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

cs_add_executable(${PROJECT_NAME}
  src/main.cc
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBRARIES}
)

cs_install()
cs_export()
