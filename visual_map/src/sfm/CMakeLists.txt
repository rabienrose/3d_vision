cmake_minimum_required(VERSION 2.8.3)
project(sfm)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#######
# APP #
#######
cs_add_executable(${PROJECT_NAME}
  src/main.cc
  src/two_frame_pose.cc
  src/ORBmatcher.cc
  src/ORBextractor.cc
)
target_link_libraries(${PROJECT_NAME}
)

############
## EXPORT ##
############
cs_install()
cs_export()
