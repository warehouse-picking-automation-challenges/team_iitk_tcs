cmake_minimum_required(VERSION 2.8.3)
project(xwamotion_laser)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  roscpp
  sensor_msgs
  std_msgs
)

add_service_files(
  FILES
  FwdBck.srv
  UpDown.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xwamotion_laser
 CATKIN_DEPENDS geometry_msgs message_runtime roscpp roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xwamotion_laser_node src/xwamotion_laser_node.cpp)

target_link_libraries(xwamotion_laser_node
  ${catkin_LIBRARIES}
)


