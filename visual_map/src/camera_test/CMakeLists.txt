cmake_minimum_required(VERSION 2.8.3)
project(camera_test)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#######
# APP #
#######
cs_add_executable(camera_test
  src/main.cc
)
target_link_libraries(${PROJECT_NAME}
)

############
## EXPORT ##
############
cs_install()
cs_export()
