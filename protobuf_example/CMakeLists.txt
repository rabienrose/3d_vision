cmake_minimum_required(VERSION 2.8.3)
project(protobuf_example)

find_package(Eigen3 REQUIRED)
add_executable(${PROJECT_NAME}
  src/main.cc
)
