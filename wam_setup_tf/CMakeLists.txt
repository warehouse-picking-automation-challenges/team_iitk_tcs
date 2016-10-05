cmake_minimum_required(VERSION 2.8.3)
project(wam_setup_tf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinect_setup_tf
#  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wam_tf src/wam_tf_broadcaster.cpp)
target_link_libraries(wam_tf ${catkin_LIBRARIES})
