cmake_minimum_required(VERSION 2.8.3)
project(iitktcs_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iitktcs_utils
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(get_bin_corners src/get_bin_corners.cpp)

 target_link_libraries(get_bin_corners
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
 )
