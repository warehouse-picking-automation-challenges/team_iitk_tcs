#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


#include <rcnn_object_detection/rcnn_object_detection_stowing.h>


using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc,argv,"fake_rcnn_img_service");

    ros::NodeHandle nh;

    ros::ServiceClient fake_service = nh.serviceClient<rcnn_object_detection::rcnn_object_detection_stowing>("/rcnn_object_detection_stowing_service");

    cv::Mat img = cv::imread("/home/nishant/apc_ws/src/fake_rcnn_img_service/bin/12.jpg");

//    cv::imshow("img",img);
//    cv::waitKey(0);

    sensor_msgs::Image ros_img_msg;
    cv_bridge::CvImage img_bridge;

    std_msgs::Header header;

    header.seq = 0;
    header.stamp = ros::Time::now();

    img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,img);
    img_bridge.toImageMsg(ros_img_msg);

//    cout << img << endl;
//    fake_rcnn_img_service::rcnn_object_detection_stowing fake_call_msg;
    rcnn_object_detection::rcnn_object_detection_stowing fake_call_msg;

//    cout << ros_img_msg.header.frame_id << endl;
//    cout << ros_img_msg.header.stamp << endl;

//    cout << ros_img_msg.height << endl;
//    cout << ros_img_msg.width << endl;


    fake_call_msg.request.input_rgb_img = ros_img_msg;

//    fake_call_msg.request.input_rgb_img.data = ros_img_msg.data;
//    fake_call_msg.request.input_rgb_img.header = ros_img_msg.header;
//    fake_call_msg.request.input_rgb_img.encoding = ros_img_msg.encoding;
//    fake_call_msg.request.input_rgb_img.height = ros_img_msg.height;
//    fake_call_msg.request.input_rgb_img.width = ros_img_msg.width;
//    fake_call_msg.request.input_rgb_img.is_bigendian = ros_img_msg.is_bigendian;
//    fake_call_msg.request.input_rgb_img.step = ros_img_msg.step;



    fake_call_msg.request.stow_obj_list.data.push_back(6);
    fake_call_msg.request.stow_obj_list.data.push_back(21);
    fake_call_msg.request.stow_obj_list.data.push_back(13);
    fake_call_msg.request.stow_obj_list.data.push_back(30);
    fake_call_msg.request.stow_obj_list.data.push_back(12);
    fake_call_msg.request.stow_obj_list.data.push_back(37);

    cout << "Calling service \t" << endl;

    if(fake_service.call(fake_call_msg))
    {
        cout << "Called successfully" << endl;

        cout << "Score of the box \t" << fake_call_msg.response.score << endl;
    }
    else
    {
        cout << "Can not call service" << endl;
    }

    return 0;
}
