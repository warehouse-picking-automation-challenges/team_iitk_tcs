#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iitktcs_msgs_srvs/fetch_foscam_ensenso_cloud.h>
#include <iitktcs_msgs_srvs/GetCloud.h>
//#include <iitktcs_msgs_srvs/paint.h>

using namespace std;
using namespace cv;
using namespace pcl;

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

bool serviceGetCloudCallback(iitktcs_msgs_srvs::GetCloud::Request &req,
                             iitktcs_msgs_srvs::GetCloud::Response &res,
                             ros::ServiceClient client, bool &pub_cloud,
                             sensor_msgs::PointCloud2 &cloudrgba_msg, sensor_msgs::Image &image_msg)
{
    cout << "Reached function to capture cloud from ensenso" << endl;
    iitktcs_msgs_srvs::fetch_foscam_ensenso_cloud fetch_cloud;
//    iitktcs_msgs_srvs::paint paint_cloud;
    fetch_cloud.request.is_organized.data = true;
    if(req.get_cloud.data)
    {
        if(client.call(fetch_cloud))
//        if(client.call(paint_cloud))
        {
            cloudrgba_msg = fetch_cloud.response.ensenso_cloud;
            res.sensor_cloud = fetch_cloud.response.ensenso_cloud;

//            cloudrgba_msg = paint_cloud.response.col_cloud;
            pub_cloud = true;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudrgba(new pcl::PointCloud<pcl::PointXYZRGBA> ());
            pcl::fromROSMsg(cloudrgba_msg, *cloudrgba);
            pcl::toROSMsg(cloudrgba_msg, image_msg);
            pcl::toROSMsg(cloudrgba_msg, res.sensor_image);

            cv::Mat image;
//            cv::namedWindow("Registered image", cv::WINDOW_NORMAL);
//            createImage(cloudrgba, image);
//            cv::imshow("Registered image", image);
//            cv::waitKey(1);
//            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
//            viewer -> addCoordinateSystem(.1);
//            viewer -> setBackgroundColor(0, 0, 0);
//            viewer -> initCameraParameters();
//            viewer -> addPointCloud(cloudrgba);
//            while(!viewer -> wasStopped())
//                viewer -> spinOnce();

            return true;
        }
        else
        {
            pub_cloud = false;
            cout << "Failed to call service: Fetch foscam ensenso cloud" << endl;
            return false;
        }
    }
    else
    {
        pub_cloud = false;
        return true;
    }
}

// code for capturing ensenso registered point cloud from either ashish or siddharth service
// once captured. The image and point cloud is published continuously on topics
// The published data can be used for semi auto calibration and for capturing bin corners
int main(int argc, char **argv)
{
    ros::init(argc, argv, "registered_cloud");
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::Image image_msg;


    bool publish_cloud = false;

    // Ashish service for getting registere cloud from ensenso
    ros::ServiceClient service_registered_cloud =
            nh.serviceClient<iitktcs_msgs_srvs::fetch_foscam_ensenso_cloud>("/iitktcs/fetch_ensenso_cloud");

    ros::ServiceServer capture_cloud_service =
            nh.advertiseService<iitktcs_msgs_srvs::GetCloud::Request, iitktcs_msgs_srvs::GetCloud::Response>
            ("/iitktcs/ensenso/registered_cloud",
             boost::bind(serviceGetCloudCallback, _1, _2, boost::ref(service_registered_cloud),
                         boost::ref(publish_cloud), boost::ref(cloud_msg), boost::ref(image_msg)));

    // Siddarth service for getting registere cloud from ensenso
//    ros::ServiceClient service_siddarth_cloud =
//            nh.serviceClient<iitktcs_msgs_srvs::paint>("/ensenso/get_cloud");


//    ros::ServiceServer capture_cloud_service =
//            nh.advertiseService<iitktcs_msgs_srvs::GetCloud::Request, iitktcs_msgs_srvs::GetCloud::Response>
//            ("/iitktcs/ensenso/registered_cloud",
//             boost::bind(serviceGetCloudCallback, _1, _2, boost::ref(service_siddarth_cloud),
//                         boost::ref(publish_cloud), boost::ref(cloud_msg), boost::ref(image_msg)));


    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/iitktcs/ensenso/depth_registered/points", 10);
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image> ("/iitktcs/foscam/image_color", 10);

//    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
//        if(publish_cloud)
//        {
//            pub_cloud.publish(cloud_msg);
//            pub_image.publish(image_msg);
//        }
//        loop_rate.sleep();
    }




    return 0;
}

