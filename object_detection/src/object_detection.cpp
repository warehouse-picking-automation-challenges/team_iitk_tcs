#include <iostream>
#include <object_detection/object_detection.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include <object_detection/stowObjDetect.h>

int main(int argc,char **argv)
{

    std::cout << "Starting object detection \t" << std::endl;

    ros::init(argc,argv,"object_Detection");



    ObjectDetection *obj_det = new ObjectDetection();
//    int p;
//    cout<<"Enter number"<<endl;
//    cin>>p;
//    cout<<"P= "<<p<<endl;

//    obj_det->sub_ptCloud = obj_det->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1,&ObjectDetection::detect_object_callback,obj_det);

//    obj_det->sub_img = obj_det->nh.subscribe<sensor_msgs::Image>("/usb/image_raw",1,&ObjectDetection::rgb_img_callback,obj_det);

    obj_det->detect_object = obj_det->nh.advertiseService("/detect_object",&ObjectDetection::detect_object_service_callback,obj_det);

    obj_det->stow_detect_object = obj_det->nh.advertiseService("/stow/detect_object",&ObjectDetection::stow_detect_object_service_callback,obj_det);

    obj_det->rcnn_service = obj_det->nh.serviceClient<object_detection::rcnn_object_detection>("/rcnn_object_detection_service",1);

    obj_det->rack_registration_service = obj_det->nh.advertiseService("/rack_registration",&ObjectDetection::rack_registration_service_callback,obj_det);

    obj_det->rcnn_stow_task_service = obj_det->nh.serviceClient<object_detection::rcnn_object_detection_stowing>("/rcnn_object_detection_stowing_service",1);



    //    pcl::visualization::CloudViewer cloud_vwr("CLOUD2");
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//    cloud_vwr.showCloud(ptr_cloud);

//    while(1);
    //    ros::spin();
//    ros::MultiThreadedSpinner spinner(4);

//    spinner.spin();

    ros::spin();

    return 0;
}
