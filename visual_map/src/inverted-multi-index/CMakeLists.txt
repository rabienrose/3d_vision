cmake_minimum_required(VERSION 2.8)
project(inverted_multi_index)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(LIBRARY_NAME ${PROJECT_NAME})

set(PROTO_DEFNS proto/inverted-multi-index/inverted_multi_index.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

cs_add_library(${LIBRARY_NAME} src/inverted-multi-index.cc
                               ${PROTO_SRCS})

cs_install()
cs_export()
