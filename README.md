# team_iitk_tcs

Description: Programs related to Amazon Picking Challenge(APC) implemented using Barrett X-WAM robot manipulator
Author: Sharath Jotawar Email: sharathrjtr@gmail.com. 
This software is being released under MIT License. https://opensource.org/licenses/MIT

-----------------------------
+ Packages: 
  - apc_controller: System architecture central process which is responsible for calling control commands to move arm, calling services to obtain trajectory planning, object detection, rack detection, read and write of json file, planning for X-WAM mobile base.
  - ik_test_service: Package for testing calibration.
  - json_maker: Provides ROS services for reading and writing json files
  - lines_rack_det: Provides service to detect rack and obtain bin centroids and corners
  - model_descriptions: urdf descriptions for X-WAM manipulator, kiva pod
  - object_detection: Package for object detection, pose estimation.
  - rcnn_object_detection: Provides service to call RCNN trained model for object detection.
  - trajectory_rpt: Package to obtain trajectory planning for pick and stow task
  - wam_setup_tf: Package to publish calibrated transformation
  - xwamotion_laser: Package to control motion of mobile base



