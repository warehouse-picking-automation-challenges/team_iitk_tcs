#include<ros/ros.h>
#include<pcl/point_types.h>
#include<pcl/common/centroid.h>
#include<pcl/io/pcd_io.h>
#include<iitktcs_msgs_srvs/static_point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<tf/tf.h>
#include<opencv2/opencv.hpp>
#include<pcl/visualization/pcl_visualizer.h>
#include<tf/transform_listener.h>
#include<eigen_conversions/eigen_msg.h>
#include<eigen_conversions/eigen_kdl.h>
#include<pcl/common/transforms.h>
#include<tf_conversions/tf_eigen.h>
#include<octomap.h>


using namespace std;
using namespace pcl;
using namespace cv;



int main(int argc,char **argv)
{

    ros::init(argc,argv,"test_static_point_cloud");
    ros::NodeHandle nh;


    cout << "load the pcd file" << endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);


    stringstream ss,ss1;

    ss << "/home/manish/new_ws/src/iitktcs_pose_estimation/data_new/" << argv[1] << ".pcd";
    ss1 << "/home/manish/new_ws/src/iitktcs_pose_estimation/data_new/" << argv[2] << ".pcd";

    pcl::io::loadPCDFile(ss.str().c_str(),*bin_cloud);

    cv::Mat image;
    createRackImageCloud(bin_cloud,image);
    extractPointCloud(bin_cloud,object_cloud,image);


    //    pcl::io::loadPCDFile(ss1.str().c_str(),*object_cloud);

    //    pcl::visualization::PCLVisualizer vis("test_cloud");
    //    vis.addPointCloud(bin_cloud);
    //    vis.spin();

    //    vis.removeAllPointClouds();

    //    vis.addPointCloud(object_cloud);
    //   vis.spin();
    //    vis.close();


    ros::ServiceClient client_static_pcd = nh.serviceClient<iitktcs_msgs_srvs::static_point_cloud>("/iitktcs/utils/octomap");
    iitktcs_msgs_srvs::static_point_cloud static_pcd;

    pcl::toROSMsg(*bin_cloud,static_pcd.request.bin_cloud);
    pcl::toROSMsg(*object_cloud,static_pcd.request.object_cloud);
    if(atoi(argv[3])==1)
        static_pcd.request.flag.data = true;
    else
        static_pcd.request.flag.data = false;


    if(client_static_pcd.call(static_pcd))
    {
        cout << "sucessfully called the service "  << endl;
        cout << "sucess parameter of service" << static_pcd.response.success.data << endl;

    }
    else
        cout << "Failed to call service" << endl;


    cout << "ENter to clear octomap" << endl;
    getchar();
    static_pcd.request.flag.data = false;


    if(client_static_pcd.call(static_pcd))
    {
        cout << "sucessfully called the service "  << endl;
        cout << "sucess parameter of service" << static_pcd.response.success.data << endl;

    }
    else
        cout << "Failed to call service" << endl;


    return 0;
}
