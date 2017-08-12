cmake_minimum_required(VERSION 2.8.3)
project(iitktcs_msgs_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  objects_info.msg
  custom_ids.msg
  grasp_pose.msg
  HandleMsg.msg
  ValleyMsg.msg
)

add_service_files(
  FILES
  computer_vision_stowing_picking.srv
  gripper_suction_controller.srv
  pose.srv
  UR5PoseGoal.srv
  UR5Goal.srv
  GetFK.srv
  TestCalibration.srv
  EnsensoGetCloud.srv
  EnsensoRegisteredCloud.srv
  Repose.srv
  Retrieval.srv
  GripperRealRviz.srv
  fetch_foscam_ensenso_cloud.srv
  GetCloud.srv
  paint.srv
  static_point_cloud.srv
  CheckClearProtectiveStop.srv
  DetectValley.srv
  DetectedValley.srv
  GripperPoseGoal.srv
  GraspPose.srv
  motor_input.srv
  publish_on_topic.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iitktcs_msgs_srvs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/iitktcs_msgs_srvs_node.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
