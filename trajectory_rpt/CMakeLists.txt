cmake_minimum_required(VERSION 2.8.3)
project(trajectory_rpt)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf
  tf_conversions
  wam_ikfast_7dof_service
)


## Generate services in the 'srv' folder
add_service_files(
    FILES
    TrajRpt.srv
    TrajStow.srv
    ContinuousTrajGen.srv
)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_rpt
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf tf_conversions wam_ikfast_7dof_service
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wam_ikfast_7dof_service_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(trajectory_rpt_node src/trajectory_rpt.cpp)
add_executable(trajectory_rpt_side src/trajectory_rpt_side.cpp)
add_executable(trajectory_stow src/trajectory_stow.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(trajectory_rpt_node ${catkin_LIBRARIES})
target_link_libraries(trajectory_rpt_side ${catkin_LIBRARIES})
target_link_libraries(trajectory_stow ${catkin_LIBRARIES})


