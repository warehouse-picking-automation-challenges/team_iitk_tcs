cmake_minimum_required(VERSION 2.8.3)
project(iitktcs_pose_estimation)

#SET(CMAKE_CXX_FLAGS "-std=c++11")
#SET("OpenCV_DIR" /home/manish/opencv/opencv-3.1.0/build)
find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        std_msgs
        geometry_msgs
        roscpp
        sensor_msgs
        message_generation
        tf
        tf_conversions
        eigen_conversions
        cv_bridge
         )


find_package(PCL 1.7 REQUIRED)
find_package(OpenCV  REQUIRED)


#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

catkin_package(
 CATKIN_DEPENDS
    DEPENDS system_lib
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    message_runtime
    message_generation
)

include_directories(
          ${catkin_INCLUDE_DIRS}
          ${PCL_INCLUDE_DIRS}
          ${OpenCV_INCLUDE_DIRS}
          include
)


#add_executable(3d_pose_estimation src/3d_pose_estimation.cpp src/3d_pose_estimation_fuction.cpp)
#target_link_libraries(3d_pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(write_yaml src/write_yaml.cpp )
#target_link_libraries(write_yaml ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


#add_executable(write_yaml_known src/write_yaml_known.cpp )
#target_link_libraries(write_yaml_known ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(writing_launch_file src/writing_launch_file.cpp )
#target_link_libraries(writing_launch_file ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
