#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <iitktcs_msgs_srvs/EnsensoRegisteredCloud.h>

using namespace std;
using namespace cv;
using namespace pcl;

#define NUM_CAPTURE_POINTS 12
#define NUM_POSITION 2

void getCloud(ros::ServiceClient &client_registered_cloud, cv::Mat &image,
              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudrgba)
{
    iitktcs_msgs_srvs::EnsensoRegisteredCloud ensenso_registered_cloud;

    cv::namedWindow("Calibrated_foscam_image", cv::WINDOW_NORMAL);

    cout << "Capturing cloud and image from ensenso" << endl;

    bool got_cloud = false;
    visualization::PCLVisualizer viewer("cloud");
    viewer.addCoordinateSystem(0.1);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();

    while(ros::ok() && !got_cloud)
    {
        ensenso_registered_cloud.request.get_cloud.data = true;

        if(client_registered_cloud.call(ensenso_registered_cloud))
        {
            while(ros::ok() && image.empty())
            {
                cout << "Waiting for image from topic: " << "/iitktcs/foscam/image_color" << endl;
                ros::spinOnce();
            }

            viewer.addPointCloud(cloudrgba);
            cout << "***** Press 'c': start capturing camera points" << endl;
            cout << "***** Press 'ESC': for next camera view" << endl;
            while(ros::ok() && !image.empty())
            {
                cv::imshow("Calibrated_foscam_image", image);
                viewer.spinOnce();

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
    viewer.close();
}

struct CapturePoints
{
    Vec2i left, right, middle;
    bool got_left, got_right, got_middle;
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    CapturePoints *cp = (CapturePoints *) userdata;

    if  ( event == EVENT_LBUTTONDOWN )
    {
        Vec2i t;
        t(0) = y;
        t(1) = x;
        cp->left = t;
        cp->got_left = true;
        cout << "Left button of the mouse is clicked - position (" << t(0) << ", " << t(1) << ")" << endl;

    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        Vec2i b;
        b(0) = y;
        b(1) = x;
        cp->right = b;
        cp->got_right = true;
        cout << "Right button of the mouse is clicked - position (" << b(0) << ", " << b(1) << ")" << endl;
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

tf::StampedTransform getSensorToBaseFrame()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);

    return transform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_sensor_robot_auto");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ());
    cv::Mat src_image;

    ros::Subscriber sub_image =
            nh.subscribe<sensor_msgs::Image>
            ("/iitktcs/foscam/image_color",1,boost::bind(imageCallback,_1,boost::ref(src_image)));
    ros::Subscriber sub_cloud =
            nh.subscribe<sensor_msgs::PointCloud2>
            ("/iitktcs/ensenso/depth_registered/points", 1,
             boost::bind(kinectPCCallback, _1, boost::ref(src_cloud)));

    //*********** Ensenso Foscam stuff **********//
    ros::ServiceClient client_ensenso_registered_cloud =
            nh.serviceClient<iitktcs_msgs_srvs::EnsensoRegisteredCloud>("/iitktcs/ensenso/registered_cloud");

    vector< vector<geometry_msgs::Point> > position_pts((int) NUM_POSITION);

    cout << "At each position capture " << (int) NUM_CAPTURE_POINTS << endl;
    cout << "Left click on mouse in the image **** Capture Points *****" << endl;
    cout << "Press Enter after taking robot to position 1" << endl;

    getchar();
    getCloud(client_ensenso_registered_cloud, src_image, src_cloud);

    cout << "Press ESC on image: restart capturing points" << endl;

    CapturePoints capture_points;
    capture_points.got_left = false;
    capture_points.got_middle = false;
    capture_points.got_right = false;
    cv::namedWindow("Capture Points", cv::WINDOW_NORMAL);
    setMouseCallback("Capture Points", CallBackFunc, (void *) &capture_points);
    vector<geometry_msgs::Point> recorded_pts;

    while(ros::ok())
    {
        ros::spinOnce();

        if(!src_image.empty())
        {
            cv::imshow("Capture Points", src_image);
            char c  = cv::waitKey(1);
            if(c==27) // if ESC then break
                recorded_pts.clear();
        }
        if(capture_points.got_left)
        {
            capture_points.got_left = false;
            PointXYZ ptxyz_left;
            point2Dto3D(capture_points.left, ptxyz_left, src_cloud);

            geometry_msgs::Point point;
            point.x = ptxyz_left.x;
            point.y = ptxyz_left.y;
            point.z = ptxyz_left.z;
            recorded_pts.push_back(point);

            if(recorded_pts.size() == (int) (NUM_CAPTURE_POINTS))
                break;
        }
    }


    cv::destroyWindow("Capture Points");


    return 0;
}
