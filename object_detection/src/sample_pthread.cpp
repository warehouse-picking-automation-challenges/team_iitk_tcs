#ifndef SAMPLE_PTHREAD_CPP
#define SAMPLE_PTHREAD_CPP

#include <iostream>
#include <pthread.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

using namespace std;

void *func1(void *arg)
{
    cv::Mat img1 = cv::imread("/home/nishant/figure_4.png");
    cv::startWindowThread();

    cv::imshow("img1",img1);
    cv::waitKey(0);
//    char c1= (char)cv::waitKey(100);
//    if( c1 == '\x1b')
//    {
//        cv::destroyWindow("img1");
//        exit(0);
//    }

//    while(1)
//    {
//        cout << "Inside fun11111" << endl;

//    }

    return NULL;
}

//void *func2(void *arg)
//{
//    cv::Mat img2 = cv::imread("/home/nishant/figure_4.png");

//    while(1)
//    {
//        cout << "Inmside fun 222 " << endl;
////        cv::imshow("img2",img2);
////        cv::waitKey(0);
//    }
//    return NULL;
//}

cv::Mat hd_cam_frame;


void rgb_hd_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cout << "Inside callback" << endl;


//        cv_bridge::CvImagePtr cv_ptr;
//        try
//        {
//            cout << "Trying to capture data" << endl;
//            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//        }
//        catch (cv_bridge::Exception& e)
//        {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return ;
//        }

//    //    cout << "Before assigning camera data to hd_cam_frame " << endl;

//        hd_cam_frame = (cv::Mat)cv_ptr->image;



        cout << hd_cam_frame << endl;
        cout << "Camera img captured \t" << endl;

    //    if(hd_cam_frame.data != NULL)
    //        hd_img_flag = true;
    //    cv::imshow("rgb",hd_cam_frame);
    //    cv::waitKey(1);

//    try
//    {
//        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//        cv::waitKey(30);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//    }
    //    call_complete = true;
    return;
}

int main(int argc, char **argv)
{

    //    cv::VideoCapture cap(1);

    //    cv::Mat frame;

    //    cap >> frame;
    ////    bool bsucess = cap.read(frame);

//    ros::init(argc,argv,"node");
//// /*   //    cout << frame << endl;
////    //    cout << frame.rows << "\t" << frame.cols << endl;


//    ros::NodeHandle n_img;
//    ros::Subscriber sub = n_img.subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,rgb_hd_img_callback);
////    //    image_transport::ImageTransport it(n_img);
//    //     image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, rgb_hd_img_callback);
//    ros::spin();*/

    cout << CV_VERSION << endl;
    cv::Mat img1 = cv::imread("/home/nishant/figure_4.png");
//    cout << img1 << endl;
    cv::imshow("img1",img1);
    cv::waitKey(0);

//    while(1)
//    {
//        ros::spinOnce();

//        if(hd_cam_frame.data != NULL)
//        {
//            cv::imshow("img",hd_cam_frame);
//            cv::waitKey(10);
//        }
//    }
    //    cv::Mat img = frame.clone();


    //    cv::Mat img = frame.clone();
    //    cap.read(frame);

    //    cv::imshow("Frame",img);
    //    cv::waitKey(0);
//        pthread_t th_1,th_2;

//        pthread_create(&th_1,NULL,func1,NULL);

//    //    pthread_create(&th_2,NULL,func2,NULL);


//        pthread_join(th_1,NULL);
    //    pthread_join(th_2,NULL);

    return 0;
}

#endif // SAMPLE_PTHREAD_CPP
