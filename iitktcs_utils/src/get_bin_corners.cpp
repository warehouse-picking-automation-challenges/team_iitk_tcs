#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/io/io.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <tf/transform_listener.h>

#include <iitktcs_msgs_srvs/GetCloud.h>

using namespace pcl;
using namespace cv;
using namespace std;
int pixel[2];
Vec2i top, bottom;

bool save = false;
bool got_point = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          top(0) = y;
          top(1) = x;
          cout << "Left button of the mouse is clicked - position (" << top(0) << ", " << top(1) << ")" << endl;
          got_point = true;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          bottom(0) = y;
          bottom(1) = x;
          cout << "Right button of the mouse is clicked - position (" << bottom(0) << ", " << bottom(1) << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
         cout << "[" << top[0] << " " << top[1] << "] ["  << bottom[0] << " " << bottom[1] << "]" << endl;
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

tf::StampedTransform getTransform(string target_frame, string source_frame)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    return transform;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_image");
    ros::NodeHandle node;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ());
    cv::Mat src_image;

    ros::Subscriber subimage =
            node.subscribe<sensor_msgs::Image>
            ("/iitktcs/foscam/image_color",1,boost::bind(imageCallback,_1,boost::ref(src_image)));
    ros::Subscriber sub_cloud =
            node.subscribe<sensor_msgs::PointCloud2>
            ("/iitktcs/ensenso/depth_registered/points", 1,
             boost::bind(kinectPCCallback, _1, boost::ref(src_cloud)));

    cout << "Database creation software" << endl;

    cv::namedWindow("Test_image", cv::WINDOW_NORMAL);
    setMouseCallback("Test_image", CallBackFunc, NULL);

    //*********** Ensenso Foscam stuff **********//
    ros::ServiceClient ensenso_pubcloud_serviceclient =
            node.serviceClient<iitktcs_msgs_srvs::GetCloud>("/iitktcs/ensenso/registered_cloud");
    iitktcs_msgs_srvs::GetCloud ensenso_pub_cloud;

    bool got_cloud = false;

    cv::namedWindow("Calibrated_foscam_image", cv::WINDOW_NORMAL);

    cout << "Capturing cloud and image from ensenso" << endl;
    while(ros::ok() && !got_cloud)
    {
        ensenso_pub_cloud.request.get_cloud.data = true;

        if(ensenso_pubcloud_serviceclient.call(ensenso_pub_cloud))
        {
            cout << "After call" << endl;
            while(ros::ok() && src_image.empty())
            {
                cout << "Waiting for image from topic: " << "/iitktcs/foscam/image_color" << endl;
                ros::spinOnce();
            }

            cout << "***** Press 'c': start capturing camera points" << endl;
            cout << "***** Press 'ESC': for next camera view" << endl;
            while(ros::ok() && !src_image.empty())
            {
                cv::imshow("Calibrated_foscam_image", src_image);

                char c = cv::waitKey(1);
                if(c=='c')
                {
                    got_cloud = true;
                    break;
                }
                else if(c==27)
                    break;
                ros::spinOnce();
            }
        }
    }
    cv::destroyWindow("Calibrated_foscam_image");
    //*********** Ensenso Foscam stuff **********//


    while(ros::ok())
    {
        ros::spinOnce();

        if(!src_image.empty())
        {

            cv::imshow("Test_image", src_image);

            char c = (char) cv::waitKey(5);

            if(got_point)
            {
                tf::StampedTransform transform = getTransform("world", "camera_link");
                pcl::PointXYZ point3d;
                point2Dto3D(top, point3d, src_cloud);
                tf::Vector3 tf_pt(point3d.x, point3d.y, point3d.z);
                tf_pt = transform*tf_pt;
                cout << tf_pt.getX() << " " << tf_pt.getY() << " " << tf_pt.getZ() << endl;

                got_point = false;
            }
        }
    }

    return 0;
}

