cmake_minimum_required(VERSION 2.8.3)
project(iitktcs_calibration)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iitktcs_calibration
#  CATKIN_DEPENDS eigen_conversions geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(calibrate_enseno_robot_semi_auto src/calibrate_enseno_robot_semi_auto.cpp)
target_link_libraries(calibrate_enseno_robot_semi_auto ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(calibrate_sensor_robot_auto_node src/calibrate_sensor_robot_auto.cpp)
target_link_libraries(calibrate_sensor_robot_auto_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(calibrate_points_plot src/calibration_points_plot.cpp)
target_link_libraries(calibrate_points_plot ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(calibration_transform_setup src/calibration_transform_setup.cpp)
target_link_libraries(calibration_transform_setup ${catkin_LIBRARIES} )

add_executable(publish_ensenso_cloud src/publish_ensenso_registered_cloud.cpp)
target_link_libraries(publish_ensenso_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_robotcamera_calibration src/test_robotcamera_calibration.cpp)
target_link_libraries(test_robotcamera_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
