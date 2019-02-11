cmake_minimum_required(VERSION 3.1)
project(protobuf_example)

set (CMAKE_CXX_STANDARD 11)

MESSAGE( STATUS ${Protobuf_SRC_ROOT_FOLDER} )

include_directories(
    ${PROJECT_SOURCE_DIR}
)

find_package(Eigen3 REQUIRED)
add_executable(${PROJECT_NAME}
    src/main.cc
    chamo_map.pb.cc
)
target_link_libraries(${PROJECT_NAME} /usr/local/lib/libprotobuf.a)
