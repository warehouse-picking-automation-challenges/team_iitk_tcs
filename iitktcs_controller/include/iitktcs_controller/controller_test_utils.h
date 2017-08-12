#ifndef CONTROLLER_TEST_UTILS_H
#define CONTROLLER_TEST_UTILS_H

#include <iitktcs_msgs_srvs/pose.h>
#include <iitktcs_msgs_srvs/UR5Goal.h>
#include <iitktcs_msgs_srvs/UR5PoseGoal.h>
#include <iitktcs_msgs_srvs/EnsensoRegisteredCloud.h>
#include <iitktcs_msgs_srvs/objects_info.h>
#include <iitktcs_msgs_srvs/pose.h>
#include <iitktcs_msgs_srvs/computer_vision_stowing_picking.h>
#include <iitktcs_msgs_srvs/GetFK.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
//#include <tf/tf.h>

#define NO_JOINTS 6

using namespace std;
using namespace cv;
using namespace pcl;

//int pixel[2];
Vec2i top, bottom;

bool got_object_bbox = false;

//void CallBackFunc(int event, int x, int y, int flags, void* userdata);

//void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr);
//void imageCallback(const sensor_msgs::ImageConstPtr image_ptr, cv::Mat &image);

//bool point2Dto3D(Vec2i pixel, PointXYZ &point3d, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ref_cloud);

//tf::StampedTransform getSensorToBaseFrame();

//geometry_msgs::Point transformSensorToWorldFrame(tf::StampedTransform &transform, geometry_msgs::Point &point,
//                                                 bool is_vector_transform = false);

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          top(0) = y;
          top(1) = x;
          cout << "Left button of the mouse is clicked - position (" << top(0) << ", " << top(1) << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          bottom(0) = y;
          bottom(1) = x;
          cout << "Right button of the mouse is clicked - position (" << bottom(0) << ", " << bottom(1) << ")" << endl;
          got_object_bbox = true;
     }
}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr)
{
    pcl::fromROSMsg(*input, *cloud_ptr);

    return;
}

void imageCallback(const sensor_msgs::ImageConstPtr image_ptr, cv::Mat &image)
{
    cv_bridge::CvImagePtr cv_image_ptr;
    try
    {
        cv_image_ptr = cv_bridge::toCvCopy(image_ptr,"bgr8");
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image=cv::Mat(cv_image_ptr->image);

//    if(show_image)
//    {
//        cv::imshow("image_callback", image);
//        cv::waitKey(0);
//    }

    return;
}

bool point2Dto3D(Vec2i pixel, PointXYZ &point3d, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &ref_cloud)
{
    int width = ref_cloud->width, height = ref_cloud->height;
    int row = pixel[0], col = pixel[1];

    for(int kernel=0; kernel<25; kernel++)
    {
        bool is_nan = true;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;

        for(int n=row-kernel; n<=row+kernel; n++)
        {
            for(int m=col-kernel; m<=col+kernel; m++)
            {
                if(n >=0 && n < height && m >= 0 && m < width)
                {
                    PointXYZRGBA& pt = ref_cloud->at(n*width+m);
                    if(!isnan(pt.x))
                    {
                        sum_x += pt.x;
                        sum_y += pt.y;
                        sum_z += pt.z;
                        count++;
                        is_nan = false;
                    }
                }
            }
        }
        if(!is_nan)
        {
            point3d.x = sum_x/count;
            point3d.y = sum_y/count;
            point3d.z = sum_z/count;
            return true;
        }
        else
            continue;

    }
    return false;

}

tf::StampedTransform getSensorToBaseFrame()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);

    return transform;
}

// Function to transform a point or vector from sensor frame to Robot world frame
// is_vector_transform = true. Then the point is assumed to be vector
// by default is_vector_transform = false. Computes transform for the point
geometry_msgs::Point transformSensorToWorldFrame(tf::StampedTransform &transform, geometry_msgs::Point &point, bool is_vector_transform = false)
{
    tf::Vector3 pt(point.x,point.y,point.z);
    tf::Vector3 tf_pt = transform*pt;

    geometry_msgs::Point tf_point;
    if(is_vector_transform)
    {
        tf::Vector3 tr_origin = transform.getOrigin();
        tf_point.x = tf_pt.getX()-tr_origin.getX();
        tf_point.y = tf_pt.getY()-tr_origin.getY();
        tf_point.z = tf_pt.getZ()-tr_origin.getZ();

        return tf_point;
    }
    else
    {
        tf_point.x = tf_pt.getX();
        tf_point.y = tf_pt.getY();
        tf_point.z = tf_pt.getZ();
        return tf_point;
    }
}

#endif // CONTROLLER_TEST_UTILS_H
