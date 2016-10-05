cmake_minimum_required(VERSION 2.8.3)
project(lines_rack_det)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  message_generation
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
    FILES
    DetectShelf.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lines_rack_det
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(lines_rack_det_node src/lines_rack_det_node.cpp src/rackdetect.cpp)

#add_dependencies(lines_rack_det_node lines_rack_det_node_generate_messages_cpp)

target_link_libraries(lines_rack_det_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
