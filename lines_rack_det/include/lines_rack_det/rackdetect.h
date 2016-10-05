#ifndef RACKDETECT_H
#define RACKDETECT_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

using namespace pcl;
using namespace std;
using namespace cv;

#define NO_BINS 12
#define DEBUG_MODE 0

#define Y_LIMIT_MAX 0.3
#define Y_LIMIT_MIN -0.6
#define HEIGHT_EXT 0.26
#define HEIGHT_MID 0.23
#define WIDTH_EXT 0.27
#define WIDTH_MID 0.3
#define RACK_WIDTH_MIN 0.8

#define ROWS 4
#define COLUMNS 3
#define H_LINES 5
#define V_LINES_REDUCTION 0.00

class RackDetect
{
public:
    RackDetect();
    bool inWorkspace(PointXYZRGBA pt);
    void createRackImageCloud(PointCloud<PointXYZRGBA>::Ptr cloud);
    void depthEdgeDetect(Mat &depthEdgeImage, int &thresh);
    void getHoughLines(Mat &image, vector<Vec4i> &lines);
    void splitVerticalHorizontal(vector<Vec4i> &all, vector<Vec4i> &vertical, vector<Vec4i> &horizontal);
    void getCornerVerticals(vector<Vec4i> &verticals, vector<Vec4i> &cluster_v);
    void getHorizontalClusters(vector<Vec4i> &horizontals, vector<Vec4i> &cluster_h);
    void getVerHorLines(vector<Vec4i> &lines, vector<Vec4i> &filter_vertical, vector<Vec4i> &filter_horizontal);
    void getIntersections(vector<Vec4i> &vl, vector<Vec4i> &hl, vector<Vec2i> &points);
    bool point2Dto3D(Vec2i pixel, PointXYZ &point3d);
    void clusterPoints(vector<Vec3d> &in_pts, vector<Vec3d> &out_pts);
    void rearrangeClusters(vector<Vec3d> &points);
    void formRackCorners(vector<Vec3d> &points);

    void rangeCrop(PointCloud<PointXYZRGBA>::Ptr &in_cloud, PointCloud<PointXYZRGBA>::Ptr &out_cloud, bool to_dense);
    void getICPTransformation(PointCloud<PointXYZRGBA>::Ptr &ref, PointCloud<PointXYZRGBA>::Ptr &src);
    void racktoBinCorners(vector<Eigen::Vector4d> &rk_crns, vector< vector<Eigen::Vector4d> > &bin_crns,
                                      vector<Eigen::Vector4d> &bin_cnt);
    void getBinPtsTransform(vector<Eigen::Vector4d> &vt);
    void printBinCorners();
    void printBinCentroids();
    void addSphereMarkers(pcl::visualization::PCLVisualizer &visualizer,
                          vector<Eigen::Vector4d> &corners, vector<Eigen::Vector4d> &centroids);

    PointCloud<PointXYZRGBA>::Ptr ref_cloud;// reference cloud with known corner points
    PointCloud<PointXYZRGBA>::Ptr src_cloud;// source cloud corresponding to which transformation is to be found
    Mat ref_image, lines_image;
    Eigen::Matrix4d transform;// transformation matrix from ref_cloud to src_cloud
    vector< vector<Eigen::Vector4d> > bin_corners;
    vector<Eigen::Vector4d> bin_centroids;
    vector< vector<Eigen::Vector4d> > tf_bin_corners;
    vector<Eigen::Vector4d> tf_bin_centroids;

    vector<Eigen::Vector4d> vertical_corners;
    vector<Eigen::Vector4d> horizontal_corners;

    vector<Eigen::Vector4d> rack_corners;
    vector<Eigen::Vector4d> tf_rack_corners;

    int no_vert_lines;
    int no_horiz_lines;

    bool get_cloud;

    string video_path;// = ros::package::getPath("lines_rack_det").append("rack_detection.avi");
    VideoWriter video;//(video_path.c_str(),CV_FOURCC('M','J','P','G'),2, Size(640,480),true);

};

#endif // RACKDETECT_H
