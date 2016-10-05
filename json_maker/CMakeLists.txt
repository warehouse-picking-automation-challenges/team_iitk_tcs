cmake_minimum_required(VERSION 2.8.3)
project(json_maker)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   get_bin_object.srv
   write_pick_status.srv
   write_stow_data.srv
   stowToteContents.srv
   getBinContents.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES json_maker
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  /home/ravi/ros_projects/apc_ws/jsoncpp/include
)


add_executable(json_maker_node src/json_maker_node.cpp)
add_executable(json_reader_writer src/json_reader_writer.cpp)

#add_dependencies(json_reader_writer json_maker_generate_messages_cpp)


# Specify libraries to link a library or executable target against
target_link_libraries(json_maker_node
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libjsoncpp.a
)
target_link_libraries(json_reader_writer
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libjsoncpp.a
)



