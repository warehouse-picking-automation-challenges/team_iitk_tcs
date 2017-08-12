#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iitktcs_msgs_srvs/TestCalibration.h>
#include <iitktcs_msgs_srvs/GetCloud.h>

using namespace cv;
using namespace std;
using namespace pcl;
typedef pcl::PointXYZRGBA Point_Type;

cv::Vec2i pixel_coord;
bool clicked = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if( event == EVENT_LBUTTONDOWN && !clicked)
    {
        cout << "Left button clicked (" << y << ", " << x << ")" << endl;
        pixel_coord(0) = y;
        pixel_coord(1) = x;
        clicked = true;
    }

    return ;
}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input,
                      pcl::PointCloud<Point_Type>::Ptr &cloud_ptr)
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

void createImage(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudrgb, cv::Mat &img)
{
    img.release();
    img.create(cloudrgb->height,cloudrgb->width,CV_8UC3);

    for(int i=0; i<cloudrgb->height; i++)
        for(int j=0; j<cloudrgb->width; j++)
        {
            pcl::PointXYZRGBA pt = cloudrgb->at(j,i);
            Eigen::Vector3i v = pt.getRGBVector3i();
            cv::Vec3b color(v(2),v(1),v(0));
            img.at<cv::Vec3b>(i,j) = color;
        }

    return;
}

bool point2Dto3D(Vec2i pixel, PointXYZ &point3d, pcl::PointCloud<Point_Type>::Ptr &cloud_ptr)
{
    int width = cloud_ptr->width, height = cloud_ptr->height;
    int row = pixel[0], col = pixel[1];

    for(int kernel=0; kernel<50; kernel++)
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
                    Point_Type& pt = cloud_ptr->at(n*width+m);
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

    transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10));
    transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    return transform;
}

bool callService(string endeff_name, cv::Mat &image, pcl::PointCloud<Point_Type>::Ptr &cloud,
                 ros::ServiceClient &service_client)
{
    std::string window = "Capture end effector point";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    setMouseCallback(window, CallBackFunc, NULL);

    bool flag_captured_pt = true;//false;
    cout << "Test with " << endeff_name << endl;
    geometry_msgs::Point endeff_pt;
    while(ros::ok())
    {
//        ros::spinOnce();
        char c;
        if(!image.empty() && !cloud->empty())
        {
            cv::imshow(window, image);
            c = waitKey(1);
        }
        if(!image.empty() && !cloud->empty() && clicked)
        {
            PointXYZ pt3d;
            if(point2Dto3D(pixel_coord, pt3d, cloud))
            {
                cout << "{" << pt3d.x << ", " << pt3d.y << ", " << pt3d.z << "}" << endl;
                clicked = false;
                flag_captured_pt = true;

                endeff_pt.x = pt3d.x;
                endeff_pt.y = pt3d.y;
                endeff_pt.z = pt3d.z;
                break;

            }
        }

        if(c == 'q' || c==27)// if q or Esc is press over image window then calibrate the transformation between kinect and robot
        {
            flag_captured_pt = false;
            break;
        }

    }
    cv::destroyWindow(window);

    if(flag_captured_pt)
    {
        tf::StampedTransform transform = getTransform("/world","/camera_link");
        tf::Vector3 pt(endeff_pt.x, endeff_pt.y, endeff_pt.z);

        tf::Vector3 tf_pt = transform*pt;
        iitktcs_msgs_srvs::TestCalibration test_calibration;
        cout << "world link pt: " << tf_pt.getX() << " "  << tf_pt.getY() << " "  << tf_pt.getZ() << endl;
        test_calibration.request.end_effector_name.data = endeff_name;
        test_calibration.request.position.x = tf_pt.getX();
        test_calibration.request.position.y = tf_pt.getY();
        test_calibration.request.position.z = tf_pt.getZ();
        if(service_client.call(test_calibration))
        {
            cout << "Successfully reached target position with " << endeff_name << endl;
            return true;
        }
        else
        {
            cout << "Failed to reach target position with " << endeff_name << endl;
            return false;
        }

    }
    else
    {
        cout << "Failed to capture the selected 3d point " << endl;
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robotcamera_calibration");
    ros::NodeHandle nh;

    pcl::PointCloud<Point_Type>::Ptr src_cloud(new pcl::PointCloud<Point_Type> ());
    cv::Mat src_image;
//    string pt_cloud_topic = "/iitktcs/ensenso/depth_registered/points";
//    string img_topic = "/iitktcs/foscam/image_color";


//    ros::Subscriber subimage = nh.subscribe<sensor_msgs::Image>(img_topic,1,boost::bind(imageCallback,_1,boost::ref(src_image)));
//    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>
//            (pt_cloud_topic, 1, boost::bind(kinectPCCallback, _1, boost::ref(src_cloud)));

    ros::ServiceClient service_test_calibration = nh.serviceClient<iitktcs_msgs_srvs::TestCalibration>
            ("/iitktcs/motion_planner/test_calibration");

    // Declare the service for ensenso to publish point cloud
    // old service to capture cloud and image. No more used
//    ros::ServiceClient ensenso_pubcloud_serviceclient =
//            nh.serviceClient<ensenso_foscam_calibration::GetCloud>("/iitktcs/ensenso/get_cloud");

    // New service from ashish for publishing cloud and image from ensenso
    ros::ServiceClient ensenso_pubcloud_serviceclient =
            nh.serviceClient<iitktcs_msgs_srvs::GetCloud>("/iitktcs/ensenso/registered_cloud");

    iitktcs_msgs_srvs::GetCloud ensenso_pub_cloud;
    ensenso_pub_cloud.request.get_cloud.data = true;

    cv::namedWindow("Calibrated_foscam_image", cv::WINDOW_NORMAL);
//    int count = 0;
    bool got_cloud = false;
    cout << "Capturing cloud and image from ensenso" << endl;
//    ros::Rate loop(30);
    while(ros::ok() && !got_cloud)
    {
        if(ensenso_pubcloud_serviceclient.call(ensenso_pub_cloud))
        {
            pcl::fromROSMsg(ensenso_pub_cloud.response.sensor_cloud, *src_cloud);
            cout << src_cloud->width << " " << src_cloud->height << endl;

            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
            viewer -> addCoordinateSystem(.1);
            viewer -> setBackgroundColor(0, 0, 0);
            viewer -> initCameraParameters();
            viewer -> addPointCloud(src_cloud);
            while(!viewer -> wasStopped())
                viewer -> spinOnce();

            createImage(src_cloud, src_image);

//            count++;
//            cout << "Count: " << count << endl;
//            while(ros::ok() && src_image.empty())
//            {
//                cout << "Waiting for image from topic: " << img_topic << endl;
//                loop.sleep();
//                ros::spinOnce();
//            }

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
            }
        }
    }
    cv::destroyWindow("Calibrated_foscam_image");

    string end_effector_name = "suction_straight";
    callService(end_effector_name, src_image, src_cloud, service_test_calibration);

//    end_effector_name = "suction_bend";
//    callService(end_effector_name, src_image, src_cloud, service_test_calibration);

//    end_effector_name = "gripper";
//    callService(end_effector_name, src_image, src_cloud, service_test_calibration);

    return 0;
}


