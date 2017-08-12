cmake_minimum_required(VERSION 2.8.3)
project(iitktcs_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  eigen_conversions
  cv_bridge
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iitktcs_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${ensenso_foscam_calibration_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(controller_test_node src/controller_test.cpp)

target_link_libraries(controller_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

#add_executable(publisher_node src/publisher.cpp)

#target_link_libraries(publisher_node
#  ${catkin_LIBRARIES}
#)

