cmake_minimum_required(VERSION 2.8.3)
project(try_undistort)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#######
# APP #
#######
cs_add_executable(try_undistort
  src/main.cc
)
target_link_libraries(${PROJECT_NAME}
)

############
## EXPORT ##
############
cs_install()
cs_export()
