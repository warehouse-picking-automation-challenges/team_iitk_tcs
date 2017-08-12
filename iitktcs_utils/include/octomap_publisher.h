#ifndef OCTOMAP_PUBLISHER_H
#define OCTOMAP_PUBLISHER_H

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
#include<std_srvs/Empty.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/search/kdtree.h>
#include<pcl/filters/filter.h>

using namespace std;
using namespace pcl;
using namespace cv;

class PublishOctomap
{
public:
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_cloud;

    PublishOctomap();
    ~PublishOctomap();



    ros::ServiceServer service_pcd;
    ros::Publisher pub_static_pcd;

    void segmentScene(PointCloud<PointXYZ>::Ptr &scene,PointCloud<PointXYZ>::Ptr &object,
                      PointCloud<PointXYZ>::Ptr &segmented_cloud);


    void staticPcd();

    void clearOctomap();

    bool serviceCallback(iitktcs_msgs_srvs::static_point_cloud::Request &req,
                         iitktcs_msgs_srvs::static_point_cloud::Response &res);



};





PublishOctomap::PublishOctomap()
{
    octomap_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());

    pub_static_pcd = nh.advertise<sensor_msgs::PointCloud2>("/iitktcs/ensenso/depth_registered/points",1000);

    service_pcd=
            nh.advertiseService("/iitktcs/utils/octomap",&PublishOctomap::serviceCallback,this);


}

PublishOctomap::~PublishOctomap()
{


}




void PublishOctomap::segmentScene(PointCloud<PointXYZ>::Ptr &scene,PointCloud<PointXYZ>::Ptr &object,PointCloud<PointXYZ>::Ptr &segmented_cloud)
{
    octomap_cloud->clear();
    vector<int> index;
    vector<int> index1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_free_scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::removeNaNFromPointCloud(*scene,*nan_free_scene,index);
    pcl::removeNaNFromPointCloud(*object,*model,index1);

    pcl::search::KdTree<pcl::PointXYZ> search;
    double dist = 0.01;
    search.setInputCloud(nan_free_scene);

    segmented_cloud->header = nan_free_scene->header;

    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    std::vector<int> indices;

    int cloud_size  = model->size();
    double time1 = ros::Time::now().toSec();

    for(int i=0;i<cloud_size;i++)
    {
        pcl::PointXYZ pt;
        pt = model->at(i);
        search.radiusSearch(pt,dist,nn_indices,nn_dists,100);
        //        search.nearestKSearch(pt,1,nn_indices,nn_dists);
        for(int j=0;j<nn_indices.size();j++)
            indices.push_back(nn_indices[j]);
    }

    cout << "Time taken:  " << ros::Time::now().toSec() - time1 <<endl;
    // add nearby indices to the segmented cloud
    //    for (int i=0;i<indices.size();i++)
    //    {
    //        segmented_cloud->points.push_back(nan_free_scene->points[indices[i]]);
    //    }

    sort(indices.begin(),indices.end());
    //    vec.erase( unique( vec.begin(), vec.end() ), vec.end() );

    indices.erase(unique( indices.begin(), indices.end() ), indices.end());

    double time = ros::Time::now().toSec();
    int w =0;
    for (int i=0;i<nan_free_scene->size();i++)
    {
        if(i == indices[w])
            w++;
        else
        {
            segmented_cloud->points.push_back(nan_free_scene->points[i]);
        }

    }
    cout << "Time taken:  " << ros::Time::now().toSec() - time <<endl;


}


void PublishOctomap::staticPcd()
{
    sensor_msgs::PointCloud2 cloud_msg;
    //    segmentScene(bin_cloud,object_cloud,octomap_cloud);

//    bin_cloud_size = octomap_cloud->size();
//    cout << "octomap cloud size: " << bin_cloud_size << endl;

    pcl::toROSMsg(*octomap_cloud,cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/camera_link";
    pub_static_pcd.publish(cloud_msg);
    sleep(1);
    cout << "Octomap cloud Published" << endl;
}

void PublishOctomap::clearOctomap()
{
    stringstream ss;
    //    ss << "rosservice call /clear_octomap " << "\"{}\"";

    //    cout << "Time taken1 : " << ros::Time::now().toSec() - time << endl;
    //    cout  << ss.str().c_str() << endl;
    //    for(int i=0;i<1;i++)
    //    {
    //        system(ss.str().c_str());
    //    }



    cout << "Reached in clear octomap function" << endl;
    sensor_msgs::PointCloud2 cloud_msg;

    double time = ros::Time::now().toSec();

    cout << "cloud size during clearnig: " << octomap_cloud->size() << endl;
    for(int i=0;i<octomap_cloud->size();i++)
    {
        octomap_cloud->points[i] = pcl::PointXYZ(10000,10000,10000);
    }


    pcl::toROSMsg(*octomap_cloud,cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/camera_link";
    pub_static_pcd.publish(cloud_msg);
    //     pub_static_pcd.publish(cloud_msg);

    //    sleep(1);
    //    cout << "Time taken1 : " << ros::Time::now().toSec() - time << endl;
    cout  << ss.str().c_str() << endl;
    for(int i=0;i<1;i++)
    {
        system(ss.str().c_str());
    }
    sleep(1);

    ros::NodeHandle nh1;
    ros::ServiceClient cl = nh1.serviceClient<std_srvs::Empty>("/clear_octomap");
    std_srvs::Empty rm;
    if(cl.call(rm))
        cout << "successfully removed the octomap" << endl;
    else
        cout << "failed to call clear octomap service" << endl;


    cout << "octomap cleared" << endl;
    cout << "Time taken2 : " << ros::Time::now().toSec() - time << endl;

    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/camera_link";
    pub_static_pcd.publish(cloud_msg);
    pub_static_pcd.publish(cloud_msg);

    octomap_cloud->clear();

}


bool PublishOctomap::serviceCallback(iitktcs_msgs_srvs::static_point_cloud::Request &req,
                                     iitktcs_msgs_srvs::static_point_cloud::Response &res)
{
//    static int count =0 ;
//    cout << "Reahed in service callback" << endl;

//    PointCloud<PointXYZ>::Ptr bin_cloud(new PointCloud<PointXYZ>());
//    PointCloud<PointXYZ>::Ptr object_cloud(new PointCloud<PointXYZ>());

//    if(!req.publish_bin_cloud.data && !req.clear_octomap_cloud.data)
//    {
//        pcl::fromROSMsg(req.bin_cloud,*bin_cloud);
//        pcl::fromROSMsg(req.object_cloud,*object_cloud);

//        stringstream ss,ss1;
//        ss << "/home/manish/olyvia_data/bin_cloud" << count << ".pcd" ;
//        ss1 << "/home/manish/olyvia_data/object_cloud" << count << ".pcd";


//        pcl::io::savePCDFile(ss.str().c_str(),*bin_cloud);
//        pcl::io::savePCDFile(ss1.str().c_str(),*object_cloud);
//        count++;

//        cout << "segmenting cloud " << endl;
//        segmentScene(bin_cloud,object_cloud,octomap_cloud);
//    }

//    if(req.publish_bin_cloud.data && !req.clear_octomap_cloud.data)
//    {
//        cout << "Publishing octomap" <<  endl;
//        staticPcd();
//    }


//    if(req.clear_octomap_cloud.data)
//    {
//        cout << "clearing octomap" << endl;
//        clearOctomap();
//    }

    return true;

}




#endif // OCTOMAP_PUBLISHER_H
