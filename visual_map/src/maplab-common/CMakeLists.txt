cmake_minimum_required(VERSION 2.8.3)
project(maplab_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include(cmake/maplab_common-extras.cmake.in)

############
# PROTOBUF #
############
set(PROTO_DEFNS proto/maplab-common/eigen.proto
                proto/maplab-common/id.proto)
PROTOBUF_CATKIN_GENERATE_CPP2("proto" PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

file(GLOB ALL_HEADER_FILES "include/*")

file(COPY ${ALL_HEADER_FILES} DESTINATION ${CATKIN_DEVEL_PREFIX}/include)

#############
# LIBRARIES #
#############
find_package (Threads)
cs_add_library(${PROJECT_NAME} src/binary-serialization.cc
                               src/breakpoints.cc
                               src/combinatorial.cc
                               src/condition.cc
                               src/cubic-spline.cc
                               src/delayed-notification.cc
                               src/feature-descriptor-ref.cc
                               src/file-lock.cc
                               src/file-logger.cc
                               src/file-system-tools.cc
                               src/geometry.cc
                               src/global-coordinate-tools.cc
                               src/gnuplot-interface.cc
                               src/gravity-provider.cc
                               src/histograms.cc
                               src/multi-threaded-progress-bar.cc
                               src/progress-bar.cc
                               src/proto-serialization-helper.cc
                               src/shared-gflags.cc
                               src/sigint-breaker.cc
                               src/stringprintf.cc
                               src/threading-helpers.cc
                               src/tridiagonal-matrix.cc
                               src/unique-id.cc
                               ${PROTO_SRCS}
                               ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${PYTHON_LIBRARIES} readline)

############
## EXPORT ##
############
cs_install()
cs_export(CFG_EXTRAS maplab_common-extras.cmake)
