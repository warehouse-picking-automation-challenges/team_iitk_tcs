cmake_minimum_required(VERSION 2.8.3)
project(apc_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    roslib
    geometry_msgs
    visualization_msgs
    wam_ikfast_7dof_service
    sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   JointMove.srv
   objectDetect.srv
   stowObjDetect.srv
   rack_registration.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
    std_msgs
    geometry_msgs # Or other packages containing msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES apc_controller
    CATKIN_DEPENDS
    DEPENDS system_lib
    roscpp
    std_msgs
    wam_srvs
    geometry_msgs
    visualization_msgs
    message_runtime
)

include_directories(
${catkin_INCLUDE_DIRS}
#${wam_srvs_INCLUDE_DIRS}
include
#${wam_ikfast_7dof_service_INCLUDE_DIRS}
#${lines_rack_det_INCLUDE_DIRS}
#${json_maker_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

add_executable(apc_controller src/apc_controller.cpp)
add_executable(apc_controller_auto src/apc_controller_auto.cpp)
#add_executable(stow_controller src/stow_controller.cpp)
add_executable(stow_controller_auto src/stow_controller_auto.cpp)

#add_dependencies(apc_controller apc_controller_generate_messages_cpp)
#add_dependencies(apc_controller_auto apc_controller_generate_messages_cpp)
#add_dependencies(stow_controller apc_controller_generate_messages_cpp)
#add_dependencies(stow_controller_auto apc_controller_generate_messages_cpp)
#add_dependencies(stow_controller_auto apc_controller_generate_messages_cpp)

target_link_libraries(apc_controller ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(apc_controller_auto ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(stow_controller ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(stow_controller_auto ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(stow_controller_auto ${catkin_LIBRARIES} ${PCL_LIBRARIES})
