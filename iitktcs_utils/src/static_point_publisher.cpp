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


void segmentScene(PointCloud<PointXYZ>::Ptr &scene,PointCloud<PointXYZ>::Ptr &object,PointCloud<PointXYZ>::Ptr &segmented_cloud)
{
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


tf::StampedTransform getSensorToBaseFrame()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);

    return transform;
}

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    tf::StampedTransform transform;
    transform = getSensorToBaseFrame();
    Eigen::Affine3d pt_transform;

    tf::transformTFToEigen(transform,pt_transform);
    pcl::transformPointCloud(*cloud,*cloud,pt_transform);

}


void PulishOctomap(PointCloud<PointXYZ>::Ptr &object_cloud,PointCloud<PointXYZ>::Ptr &bin_cloud,
                   PointCloud<PointXYZ>::Ptr &octomap_cloud,ros::Publisher &pub_static_pcd,int &bin_cloud_size)
{
    sensor_msgs::PointCloud2 cloud_msg;
    segmentScene(bin_cloud,object_cloud,octomap_cloud);

    bin_cloud_size = octomap_cloud->size();
    cout << "octomap cloud size: " << bin_cloud_size << endl;

    pcl::toROSMsg(*octomap_cloud,cloud_msg);
   cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "/camera_link";
    pub_static_pcd.publish(cloud_msg);
//    sleep(1);

    cout << "Octomap cloud Published" << endl;
}

void clearOctomap(PointCloud<PointXYZ>::Ptr &octomap_cloud,ros::Publisher &pub_static_pcd, int &cloud_size)
{
    stringstream ss;
//    ss << "rosservice call /clear_octomap " << "\"{}\"";

//    cout << "Time taken1 : " << ros::Time::now().toSec() - time << endl;
//    cout  << ss.str().c_str() << endl;
//    for(int i=0;i<1;i++)
//    {
//        system(ss.str().c_str());
//    }



    cout << "Clearing OCtomap" << endl;
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

    ros::NodeHandle nh;
    ros::ServiceClient cl = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
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
//    octomap_cloud->resize(cloud_size);
}


bool serviceCallback(iitktcs_msgs_srvs::static_point_cloud::Request &req,iitktcs_msgs_srvs::static_point_cloud::Response &res,
                     ros::Publisher &pub_static_pcd, int &bin_cloud_size,PointCloud<PointXYZ>::Ptr &octomap_cloud)

{/*
//    pcl::PointCloud<pcl::PointXYZ>::Ptr octomap_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(req.flag.data)
    {
        cout << "coping cloud" << endl;
        pcl::fromROSMsg(req.bin_cloud,*bin_cloud);
        pcl::fromROSMsg(req.object_cloud,*object_cloud);

        pcl::io::savePCDFile("/home/manish/bin_cloud.pcd", *bin_cloud);
        pcl::io::savePCDFile("/home/manish/object_cloud.pcd",*object_cloud);
       //        bin_cloud_size = bin_cloud->size();
        cout << "bin cloud size: " << bin_cloud_size << endl;
        if(bin_cloud->size() !=0 && object_cloud->size()!=0)
        {
            PulishOctomap(object_cloud,bin_cloud,octomap_cloud,pub_static_pcd,bin_cloud_size);

        }

        res.success.data  =true;
    }

    else
    {
        cout << "Have to publish blank cloud " << endl;
        cout << "octomap cloud size: " << octomap_cloud->size() << endl;
        clearOctomap(octomap_cloud,pub_static_pcd, bin_cloud_size);
        res.success.data = true;
    }*/

    return true;


}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"static_point_cloud_publisher");
    ros::NodeHandle nh;
    cout << "octomap is going to rock " << endl;

    ros::ServiceServer service_static_pcd;
    ros::Publisher pub_static_pcd;
    ros::ServiceClient octamapClear;
    PointCloud<PointXYZ>::Ptr octomap_cloud(new PointCloud<PointXYZ>);

    pub_static_pcd = nh.advertise<sensor_msgs::PointCloud2>("/iitktcs/ensenso/depth_registered/points",1000);
    int octomap_cloud_size=100000;

    bool get_cloud = false;
    bool flag = false;

    service_static_pcd=
            nh.advertiseService<iitktcs_msgs_srvs::static_point_cloud::Request,iitktcs_msgs_srvs::static_point_cloud::Response>
            ("/iitktcs/utils/octomap",boost::bind(serviceCallback, _1, _2 ,boost::ref(pub_static_pcd), boost::ref(octomap_cloud_size)
                                                  ,boost::ref(octomap_cloud)/*,boost::ref(get_cloud),boost::ref(flag)*/));


    ros::spin();

    return 0;
}



