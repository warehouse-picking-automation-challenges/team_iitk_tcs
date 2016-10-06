#include <iostream>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

using namespace std;

ros::CallbackQueue queue_rack_service;

ros::CallbackQueue queue_detect_object_service;

ros::CallbackQueue queue_point_cloud_subscriber;

void *fun_thread_rack_service(void *arg)
{

    while(1)
    {
        cout << "Rack service \t" << endl;
        queue_rack_service.callOne(ros::WallDuration(5));

    }
}

void *fun_thread_detect_object(void *arg)
{
    while(1)
    {
        cout << "Object detect service \t" << endl;
        queue_detect_object_service.callOne(ros::WallDuration(5));
    }

}

void *fun_thread_point_cloud_subscriber(void *arg)
{
    while(1)
    {
        cout << "Point cloud service \t" << endl;
        queue_point_cloud_subscriber.callOne(ros::WallDuration(5));
    }

}

int main(int argc, char **argv)
{

    ros::init(argc,argv,"object_detection_pthread");


    ros::NodeHandle nh_rack_service;

    ros::NodeHandle nh_detect_object_service;

    ros::NodeHandle nh_point_cloud_subscriber;




    nh_rack_service.setCallbackQueue(&queue_rack_service);

    nh_detect_object_service.setCallbackQueue(&queue_detect_object_service);

    nh_point_cloud_subscriber.setCallbackQueue(&queue_point_cloud_subscriber);




//    ObjectDetection *obj_det = new ObjectDetection();

//    obj_det->sub_ptCloud = obj_det->nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1,&ObjectDetection::detect_object_callback,obj_det);

//    //    obj_det->sub_img = obj_det->nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_color",1,&ObjectDetection::rgb_img_callback,obj_det);

//    obj_det->detect_object = obj_det->nh.advertiseService("/detect_object",&ObjectDetection::detect_object_service_callback,obj_det);

//    obj_det->rcnn_service = obj_det->nh.serviceClient<object_detection::rcnn_object_detection>("/rcnn_object_detection_service",1);

//    obj_det->rack_registration_service = obj_det->nh.advertiseService("/rack_registration",&ObjectDetection::rack_registration_service_callback,obj_det);


    pthread_t thread_rack_service_id,thread_detect_object_id,thread_point_cloud_id;

    pthread_create(&thread_rack_service_id,NULL,fun_thread_rack_service,NULL);
    pthread_create(&thread_detect_object_id,NULL,fun_thread_detect_object,NULL);
    pthread_create(&thread_point_cloud_id,NULL,fun_thread_point_cloud_subscriber,NULL);

    pthread_join(thread_rack_service_id,NULL);
    pthread_join(thread_detect_object_id,NULL);
    pthread_join(thread_point_cloud_id,NULL);


    return 0;
}
