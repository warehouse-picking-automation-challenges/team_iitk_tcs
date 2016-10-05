cmake_minimum_required(VERSION 2.8.3)
project(ik_test_service)

find_package(catkin REQUIRED COMPONENTS
#    cmake_modules
    tf
    tf_conversions
    roscpp
    std_msgs
    message_generation
)

add_service_files(
    FILES
    PoseJoint7dof.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ik_test_service
 CATKIN_DEPENDS
    roscpp
    tf
    tf_conversions
    std_msgs
    message_runtime
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES src/ik_test_service_node.cpp
   )

add_executable(ik_test_service_node ${SOURCES})

#add_dependencies(ik_test_service_node ik_test_service_node_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ik_test_service_node
  ${catkin_LIBRARIES}
)
