#ifndef ESTIMTION_POSE_H
#define ESTIMTION_POSE_H

#include<ros/ros.h>
#include<ros/package.h>
#include<iostream>
#include<vector>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/registration/icp.h>
#include<opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>
#include<pcl/filters/filter.h>
#include<pcl/common/transforms.h>
#include<pcl/common/centroid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<Eigen/Dense>
#include<pcl/io/pcd_io.h>
#include<pcl/registration/icp.h>
#include<pcl/common/pca.h>
#include<boost/filesystem.hpp>
#include <pcl/filters/voxel_grid.h>
#include<pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<ensenso_testing/pose.h>
#include<pcl_conversions/pcl_conversions.h>
#include<ensenso_testing/objects_info.h>
#include <string>
#include<std_msgs/Int16.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/console/time.h>
#include<pcl/features/feature.h>
#include<algorithm>
#include<tf/tf.h>
#include<eigen_conversions/eigen_msg.h>
#include<ensenso_testing/repose.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include<tf/transform_listener.h>
#include<tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#define TRANSFORM 1
#define PI 3.14
#define K 300
#define before_normal 0.10
#define bellow_length 0.05

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;
using namespace std;
using namespace pcl;
using namespace cv;
class EstimatePose
{
public:
    ros::NodeHandle nh;
    ros::ServiceServer model_fit;
    ros::ServiceServer repose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene,input_scene;
    EstimatePose();
    ~EstimatePose();
    string modelName,path,model_path;
    int geometry_type,object_type,primitive_type;

    double overlap;
    vector<int> data_normal;
    vector<double> dimensions;
    stringstream ss_model,ss_model_obj,ss_scene_obj,ss_super4pcs,ss_super4pcs_matrix;
    Eigen::Affine3d mat_super4pcs;
    Eigen::Vector3f final_centroid,final_repose_centroid;
    Eigen::Vector3f final_axis,final_axis1,final_repose_axis;
    Eigen::Vector3f final_normal,final_repose_normal;
    search::KdTree<PointXYZ>::Ptr tree;
    vector<float> bin_corner0;
    vector<float> bin_corner1;
    Eigen::Vector3f left_plane,right_plane,top_plane,bottom_plane,bin_centoid;
    vector<string> model_names;
    int object_id;
    int bin_id;
    double centroid_offset;
    double angle;
    double height_offset,width_offset,length_offset;
    vector<int> pointIdxNKNSearch;
    void getParameters();
    bool mainFuctionFormable();
    bool mainFuctionDeFormable();
    void transformPointCloud();
    void createObjFile(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,string a);
    void downSampleCloudVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud
                              ,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr,double* leafsize);
    void applySuper4pcs();
    void searchKNN(Eigen::Vector3f &model_centroid, Eigen::Vector3f &scene_centroid);
    void showPointCloud(PointCloud<PointXYZ>::Ptr &model,PointCloud<PointXYZ>::Ptr &scene);
    void planFit(PointCloud<PointXYZ>::Ptr &scene,PointCloud<Normal>::Ptr &scene_normals
                 ,Eigen::Vector3f &Normal);

    void cylinderFit(PointCloud<PointXYZ>::Ptr &scene,PointCloud<Normal>::Ptr &scene_normals
                     ,Eigen::Vector3f &Normal);

    void sphereFit(PointCloud<PointXYZ>::Ptr &scene,PointCloud<Normal>::Ptr &scene_normals
                   ,Eigen::Vector3f &Normal);

    void segmentScene(PointCloud<PointXYZ>::Ptr &nan_free_scene,PointCloud<PointXYZ>::Ptr &model
                      ,PointCloud<PointXYZ>::Ptr &segmented_cloud);

    void computeNormal(PointCloud<PointXYZ>::Ptr &scene,PointCloud<Normal>::Ptr &scene_normal);

    void normalPoint(PointCloud<PointXYZ>::Ptr &scene,Eigen::Vector4f &plane_parameter);
    void cylinderNormal(PointCloud<PointXYZ>::Ptr &scene,ModelCoefficients::Ptr &coe
                        ,Eigen::Vector3f &Normal);

    void projection(PointCloud<PointXYZ>::Ptr &object1,ModelCoefficients::Ptr &coe);
    void arrowvisulizer(PointCloud<PointXYZ>::Ptr &transformed_model,Eigen::Vector3f &model_major_axis
                        , visualization::PCLVisualizer::Ptr &visu,string a);

    double projection_distance(Eigen::Vector3f &Normal, Eigen::Vector3f &point_plane,
                               Eigen::Vector3f &given_point);

    void addsphere(pcl::visualization::PCLVisualizer::Ptr &visu,vector<Eigen::Vector3f> &points
                   , string id);

    void reposeCalculation();
    void newcode(pcl::PointCloud<pcl::PointXYZ>::Ptr &model);
    void addaxis(pcl::visualization::PCLVisualizer::Ptr &visu,Eigen::Vector3f &axis,
                 Eigen::Vector3f &centroid,double *color,string id);

    void specialCasetray(pcl::PointCloud<pcl::PointXYZ>::Ptr &model);
    void specialCaseGlass();
    void specialCaseMeshCup();
    void specialCasebrush(PointCloud<PointXYZ>::Ptr &model);
    double calDistance(Eigen::Vector3f &centroid);

    int getRegion(Eigen::Vector3f &centroid,pcl::visualization::PCLVisualizer::Ptr &visu);
    bool callBack(ensenso_testing::pose::Request &req,ensenso_testing::pose::Response &res);


    tf::StampedTransform getSensorToBaseFrame();

};


#endif // 3D_POSE_ESTIMTION_H
