#include<iostream>
#include<ctime>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<pcl/point_cloud.h>

#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>

#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>

#include<pcl/kdtree/kdtree.h>

#include<pcl/common/pca.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include<pcl/filters/passthrough.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/approximate_voxel_grid.h>
#include<pcl/filters/extract_indices.h>

#include<pcl/features/normal_3d.h>
#include<pcl/features/principal_curvatures.h>
#include<pcl/features/intensity_gradient.h>

#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/supervoxel_clustering.h>
#include<pcl/segmentation/region_growing.h>
#include<pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include<pcl/segmentation/region_growing_rgb.h>

#include<pcl/registration/transformation_estimation_svd.h>
#include<pcl/registration/icp_nl.h>
#include<pcl/registration/icp.h>





#include <object_detection_processing_routines/Model_details.h>
#include <object_detection_processing_routines/SHELF_DIMENSIONS.h>

#include<object_detection_processing_routines/point_cloud_processing.h>
#include<object_detection_processing_routines/feature_computing.h>
#include<object_detection_processing_routines/read_histograms.h>
#include<object_detection_processing_routines/cloud_registration.h>
#include<object_detection_processing_routines/back_projection_routines.h>
#include<object_detection_processing_routines/random_forest_routines.h>

#include<object_detection_processing_routines/stow_region_selection.h>



#define COLOR_IMAGE_WIDTH 640
#define COLOR_IMAGE_HEIGHT 480

//----DETECT OBJECT



void unorganized_backprojection_to_organized_backprojection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud,
                                                            std::vector<cv::Mat>& vector_unorganized_posteriors,std::vector<cv::Mat>& vector_organized_posteriors,
                                                            cv::Rect2i& rect_croped_image)
{
    for(int i=0;i<vector_unorganized_posteriors.size();i++)
    {
        vector_organized_posteriors.push_back(cv::Mat::zeros(rect_croped_image.height,rect_croped_image.width,CV_32FC1));
        for(int j=0;j<ptr_pixel_cloud->size();j++)
        {
            int loc_x = ptr_pixel_cloud->at(j).x;
            int loc_y = ptr_pixel_cloud->at(j).y;
            vector_organized_posteriors[i].at<float>(loc_y,loc_x) =  vector_unorganized_posteriors[i].at<float>(j);
        }
    }
}




void region_growing_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_bin_cloud_RGB,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_object_pixel_cloud_RGB,
                        int index_seed_point,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_object_surface_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ptr_bin_cloud_RGB,*ptr_bin_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr ptr_bin_normal_cloud(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ptr_kd_tree->setInputCloud(ptr_bin_cloud);

    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(ptr_kd_tree);
    normal_estimator.setKSearch(30);
    normal_estimator.setInputCloud(ptr_bin_cloud);
    normal_estimator.compute(*ptr_bin_normal_cloud);


    pcl::RegionGrowing<pcl::PointXYZ,pcl::Normal> region_grow_xyz;
    region_grow_xyz.setSearchMethod(ptr_kd_tree);
    region_grow_xyz.setNumberOfNeighbours(30);
    region_grow_xyz.setMinClusterSize(20);
    region_grow_xyz.setMaxClusterSize(100000);
    region_grow_xyz.setSmoothnessThreshold(20 /180.0f * M_PI);
    region_grow_xyz.setCurvatureThreshold(1.0);

    region_grow_xyz.setInputNormals(ptr_bin_normal_cloud);
    region_grow_xyz.setInputCloud(ptr_bin_cloud);

    pcl::PointIndicesPtr ptr_point_indices_cluster(new pcl::PointIndices);
    region_grow_xyz.getSegmentFromPoint(index_seed_point,*ptr_point_indices_cluster);

    std::cout << "POINT INDICES SIZE = " << ptr_point_indices_cluster->indices.size() << std::endl;

    pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor;
    indices_extractor.setInputCloud(ptr_bin_cloud_RGB);
    indices_extractor.setIndices(ptr_point_indices_cluster);
    indices_extractor.filter(*ptr_object_surface_cloud);
    indices_extractor.setInputCloud(ptr_object_pixel_cloud_RGB);
    indices_extractor.filter(*ptr_object_pixel_cloud_RGB);

    //    pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
    //    while(1)
    //    {
    //        cloud_viewer.showCloud(ptr_bin_cloud_RGB);sleep(1);
    //        cloud_viewer.showCloud(ptr_colored_region_cloud);sleep(1);
    //        cloud_viewer.showCloud(ptr_object_surface_cloud);sleep(1);
    //    }
}




void region_growing_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_bin_cloud_RGB,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_object_pixel_cloud_RGB,
                        int index_seed_point,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_object_surface_cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr ptr_bin_normal_cloud(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normal_estimator;
    normal_estimator.setKSearch(40);
    normal_estimator.setInputCloud(ptr_bin_cloud_RGB);
    normal_estimator.compute(*ptr_bin_normal_cloud);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB,pcl::Normal> region_grow_rgb;
    region_grow_rgb.setInputNormals(ptr_bin_normal_cloud);
    region_grow_rgb.setCurvatureThreshold(2.0);
    region_grow_rgb.setSmoothnessThreshold(10 /180.0f * M_1_PI);
    region_grow_rgb.setMinClusterSize(20);
    region_grow_rgb.setMaxClusterSize(30000);
    region_grow_rgb.setNumberOfNeighbours(30);
    region_grow_rgb.setInputCloud(ptr_bin_cloud_RGB);
    region_grow_rgb.setPointColorThreshold(20);
    region_grow_rgb.setRegionColorThreshold(40);

    pcl::PointIndicesPtr ptr_point_indices_cluster(new pcl::PointIndices);
    region_grow_rgb.getSegmentFromPoint(index_seed_point,*ptr_point_indices_cluster);

    pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor;
    indices_extractor.setInputCloud(ptr_bin_cloud_RGB);
    indices_extractor.setIndices(ptr_point_indices_cluster);
    indices_extractor.filter(*ptr_object_surface_cloud);
    indices_extractor.setInputCloud(ptr_object_pixel_cloud_RGB);
    indices_extractor.filter(*ptr_object_pixel_cloud_RGB);


}



bool recognize_object(std::vector<cv::Rect2i>& windows, std::vector<double>& scores_windows,
                      std::vector<int>& bin_members, cv::Mat& image,cv::Rect2i& rect_cropped_image,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_object_surface_cloud_RGB,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_object_boundary_cloud_RGB,
                      pcl::Normal& centroid_normal,
                      std::string database_path,std::string time_stamp, cv::Mat &rcnn_recog_img, cv::Mat &post_pro_img)
{
    std::cout << "reached in recognition object" << std::endl;

    for(int i=0;i < bin_members.size();i++)
        bin_members[i] -= 1;

    std::cout <<   "bin_members = "  << "   " ;
    for(int i=0;i < bin_members.size();i++)
        std::cout <<   bin_members[i]  << "   " ;
    std::cout << std::endl;

    //    pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
    //    while(1)
    //    {
    //        cloud_viewer.showCloud(ptr_bin_cloud_RGB);
    //        sleep(1);
    //    }

    std::vector<cv::Mat> vector_back_projections;
//    get_all_back_projections(bin_members,ptr_bin_cloud_RGB,vector_back_projections,database_path);

//    bool USE_DEPTH_MEAN_VAR = LOOKUP_POSTERIORS[bin_members[0] * TOTAL_NO_OF_BACKPROJECTIONS + CONST_MEAN_VAR];
//    bool USE_DEPTH_PRINCIPLE_CURVATURE = LOOKUP_POSTERIORS[bin_members[0] * TOTAL_NO_OF_BACKPROJECTIONS + CONST_PRINCIPLE_CURVATURE];
//    bool USE_TRUE_COLOR = LOOKUP_POSTERIORS[bin_members[0] * TOTAL_NO_OF_BACKPROJECTIONS + CONST_TRUE_COLOR];
//    bool USE_GRAY_SHADES = LOOKUP_POSTERIORS[bin_members[0] * TOTAL_NO_OF_BACKPROJECTIONS + CONST_GRAY_SHADES];
//    bool USE_RANDOM_FOREST = LOOKUP_POSTERIORS[bin_members[0] * TOTAL_NO_OF_BACKPROJECTIONS + CONST_RANDOM_FOREST];

//        USE_RANDOM_FOREST = false;
//    if(USE_RANDOM_FOREST)
//    {
//        vector_back_projections.push_back(cv::Mat());
//        get_random_forest_posterior(bin_members,ptr_bin_cloud_RGB,vector_back_projections[vector_back_projections.size()-1],
//                database_path);
//    }

//    std::vector<cv::Mat> vector_organized_back_projections;
//    unorganized_backprojection_to_organized_backprojection(ptr_pixel_cloud_RGB,
//                                                           vector_back_projections,vector_organized_back_projections,rect_cropped_image);

//    for(int i=0;i< vector_organized_back_projections.size();i++ )
//        cv::GaussianBlur(vector_organized_back_projections[i],vector_organized_back_projections[i],cv::Size(5,5),8,8);

    //    while(1)
    //    {
    //        for(int i=0;i< vector_organized_back_projections.size();i++)
    //        {
    //           std::stringstream window_name;
    //           window_name << "IMAGE " << i;
    //           cv::imshow(window_name.str(),vector_organized_back_projections[i]);
    //           cv::waitKey(1);
    //        }
    //    }
    //
    // there wiil be five posteriors
    // mean_var -  principle curvature - true color - gray scale - random forest

//    cv::Mat final_posterior=cv::Mat::zeros(rect_cropped_image.height,rect_cropped_image.width,CV_32FC1);

//    if(USE_DEPTH_MEAN_VAR && USE_DEPTH_PRINCIPLE_CURVATURE)
//    {
//        final_posterior += vector_organized_back_projections[CONST_MEAN_VAR];
//        final_posterior += vector_organized_back_projections[CONST_PRINCIPLE_CURVATURE];
//    }

//    final_posterior = final_posterior/2;

//    if(USE_TRUE_COLOR)
//        cv::multiply(final_posterior,vector_organized_back_projections[CONST_TRUE_COLOR],final_posterior);
//    if(USE_GRAY_SHADES)
//        cv::multiply(final_posterior,vector_organized_back_projections[CONST_GRAY_SHADES],final_posterior);
//    if(USE_RANDOM_FOREST)
//        cv::multiply(final_posterior,vector_organized_back_projections[CONST_RANDOM_FOREST],final_posterior);

//    std::cout << "POSTERIOR DONE" << std::endl;

    cv::Rect2i object_window;
    object_window.x = 0;
    object_window.y = 0;
    object_window.width = windows[0].width;
    object_window.height = windows[0].height;

//    std::cout << "MEANSHIFT START" << std::endl;

//    cv::Mat meanshift_posterior;
//    final_posterior(windows[0]).copyTo(meanshift_posterior);
//    //final_posterior.convertTo(final_posterior,CV_8UC1,255);

//    std::cout << "MEANSHIFT START" << std::endl;
//    cv::meanShift(meanshift_posterior,object_window,cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER,20,0.001));
//    std::cout << "MEANSHIFT DONE" << std::endl;

    object_window.x += windows[0].x;
    object_window.y += windows[0].y;

    cv::Point2f centroid_2d;
    centroid_2d.x = object_window.x+ object_window.width/2;
    centroid_2d.y = object_window.y+ object_window.height/2;

    std::cout << "OBJECT CENTROID" << centroid_2d  << std::endl;

    cv::Mat object_location_window;
    cv::Rect2f object_location_rectangle;

    object_location_rectangle.x = centroid_2d.x-object_window.width/2;
    object_location_rectangle.y = centroid_2d.y-object_window.height/2;
    object_location_rectangle.width = object_window.width;
    object_location_rectangle.height  = object_window.height;

    image(rect_cropped_image).copyTo(object_location_window);

    cv::rectangle(object_location_window,object_location_rectangle,cv::Scalar(0,0,255),3);
    cv::circle(object_location_window,centroid_2d,4,cv::Scalar(0,255,0),-1);


    pcl::PointXYZRGB centroid_3d;
    centroid_3d.x =0.0f;
    centroid_3d.y =0.0f;
    centroid_3d.z =0.0f;
    centroid_3d.r =0;
    centroid_3d.g =255;
    centroid_3d.b =0;


    int index_seed_point = -1;
    float min_dist = 100000;

    for(int i=0;i<ptr_pixel_cloud_RGB->size();i++)
    {
        float dist_x =(ptr_pixel_cloud_RGB->at(i).x - centroid_2d.x);
        float dist_y =(ptr_pixel_cloud_RGB->at(i).y - centroid_2d.y);
        float dist  = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        if( dist < min_dist)
        {
            min_dist = dist;
            index_seed_point = i;
        }
    }


    if(index_seed_point==-1)
        return false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_object_pixel_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*ptr_pixel_cloud_RGB,*ptr_object_pixel_cloud_RGB);

    bool use_region_growing_rgb = true;

    if(use_region_growing_rgb)
        region_growing_rgb(ptr_bin_cloud_RGB,ptr_object_pixel_cloud_RGB,index_seed_point,ptr_object_surface_cloud_RGB);
    else
        region_growing_xyz(ptr_bin_cloud_RGB,ptr_object_pixel_cloud_RGB,index_seed_point,ptr_object_surface_cloud_RGB);


    filter_point_cloud(ptr_object_surface_cloud_RGB,ptr_object_surface_cloud_RGB,
                       ptr_object_pixel_cloud_RGB,false,
                       true,true,NULL,NULL, 1.2,7);



    float min_x =  10000;
    float max_x = -10000;
    float min_y =  10000;
    float max_y = -10000;

    int loc_min_x =  0;
    int loc_max_x =  0;
    int loc_min_y =  0;
    int loc_max_y =  0;

    centroid_2d.x =0;
    centroid_2d.y =0;

    pcl::PointXYZRGB centroid_3d_RGB;


    for(int i=0;i < ptr_object_pixel_cloud_RGB->size();i++)
    {
        if(ptr_object_pixel_cloud_RGB->at(i).x < min_x )
        {
            min_x = ptr_object_pixel_cloud_RGB->at(i).x;
            loc_min_x = i;
        }
        if(ptr_object_pixel_cloud_RGB->at(i).x > max_x )
        {
            max_x = ptr_object_pixel_cloud_RGB->at(i).x;
            loc_max_x =i;
        }

        if(ptr_object_pixel_cloud_RGB->at(i).y < min_y )
        {
            min_y = ptr_object_pixel_cloud_RGB->at(i).y;
            loc_min_y = i;
        }
        if(ptr_object_pixel_cloud_RGB->at(i).y > max_y)
        {
            max_y = ptr_object_pixel_cloud_RGB->at(i).y;
            loc_max_y =i;
        }

        centroid_3d_RGB.x += ptr_object_surface_cloud_RGB->at(i).x;
        centroid_3d_RGB.y += ptr_object_surface_cloud_RGB->at(i).y;
        centroid_3d_RGB.z += ptr_object_surface_cloud_RGB->at(i).z;
    }

    centroid_3d_RGB.x /= ptr_object_surface_cloud_RGB->size();
    centroid_3d_RGB.y /= ptr_object_surface_cloud_RGB->size();
    centroid_3d_RGB.z /= ptr_object_surface_cloud_RGB->size();

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ptr_kd_tree->setInputCloud(ptr_object_surface_cloud_RGB);

    std::vector<int> vector_indices;
    std::vector<float> vector_distances;
    ptr_kd_tree->nearestKSearch(centroid_3d_RGB,1,vector_indices,vector_distances);

    index_seed_point = vector_indices[0];

    //    pcl::PointXYZRGB point;
    //    point.r =255;
    //    point.g =255;
    //    point.b =255;

    //    point.x= ptr_object_surface_cloud_RGB->at(loc_min_x);

    pcl::PointCloud<pcl::Normal>::Ptr ptr_object_surface_normal_cloud_RGB(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normal_estimator;
    normal_estimator.setKSearch(40);
    normal_estimator.setInputCloud(ptr_object_surface_cloud_RGB);
    normal_estimator.compute(*ptr_object_surface_normal_cloud_RGB);

    std::cout << "INDEX SEED POINT = " << index_seed_point << std::endl;
    centroid_normal = ptr_object_surface_normal_cloud_RGB->at(index_seed_point);


    std::vector<cv::Point2f> bounding_rect_points;
    for(int i=0;i<ptr_object_pixel_cloud_RGB->size();i++)
    {
        cv::Point2f point_2d;
        point_2d.x = ptr_object_pixel_cloud_RGB->at(i).x;
        point_2d.y = ptr_object_pixel_cloud_RGB->at(i).y;
        bounding_rect_points.push_back(point_2d);
    }


    cv::Point2f new_centroid_2d;
    new_centroid_2d.x = ptr_object_pixel_cloud_RGB->at(index_seed_point).x;
    new_centroid_2d.y = ptr_object_pixel_cloud_RGB->at(index_seed_point).y;


    cv::Rect2i new_object_window;
    new_object_window = cv::boundingRect(bounding_rect_points);

    std::cout << "NEW BOUNDING RECT = " << new_object_window << std::endl;

    cv::Mat new_object_centroid_window;
    image(rect_cropped_image).copyTo(new_object_centroid_window);

    cv::rectangle(new_object_centroid_window,new_object_window,cv::Scalar(0,0,255),3);
    cv::circle(new_object_centroid_window,new_centroid_2d,4,cv::Scalar(0,255,0),-1);

    bool write_images_to_disk = true;

    std::string images_names[] ={
        "final_posterior","color image",
        "object location image","backprojection",
        "new object location image"
    };

    // newly add following 2lines to display images on rviz
    object_location_window.copyTo(rcnn_recog_img);
    new_object_centroid_window.copyTo(post_pro_img);

    if(write_images_to_disk)
    {
       std::cout << "SAVING IMAGES" << std::endl;
//       cv::imwrite(database_path + "/image_database/internal_images/" + images_names[0] + time_stamp + ".jpg", final_posterior);
       cv::imwrite(database_path + "/image_database/internal_images/" + images_names[1] + time_stamp + ".jpg", image(rect_cropped_image));
       cv::imwrite(database_path + "/image_database/internal_images/" + images_names[2] + time_stamp + ".jpg", object_location_window);

       std::cout << "SAVING BACK PROJECTIONS" << std::endl;

//       for(int i=0;i< vector_organized_back_projections.size();i++)
//       {
//           std::stringstream window_name;
//           window_name << database_path << "/image_database/internal_images/" <<  images_names[3] << time_stamp << i << ".jpg";
//           cv::imwrite( window_name.str(), vector_organized_back_projections[i]);
//       }
       cv::imwrite(database_path + "/image_database/internal_images/" + images_names[4] + time_stamp + ".jpg",new_object_centroid_window);

   std::stringstream bin_cloud_file_name;
   std::stringstream bin_pixel_cloud_file_name;

   std::cout << "SAVING CLOUDS" << std::endl;
   bin_cloud_file_name <<  database_path << "/image_database/internal_images/" << "bin_cloud_" << time_stamp << ".ply" ;
   bin_pixel_cloud_file_name <<  database_path << "/image_database/internal_images/" << "bin_pixel_cloud_" << time_stamp << ".ply" ;


   // comment
//       pcl::io::savePLYFileBinary(bin_cloud_file_name.str(),*ptr_bin_cloud_RGB);
//       pcl::io::savePLYFileBinary(bin_pixel_cloud_file_name.str(),*ptr_pixel_cloud_RGB);
    }

//    cv::imshow("FINAL POSTERIOR",final_posterior);
//    cv::waitKey(1);
//    cv::imshow("color imgae",image(rect_cropped_image));
//    cv::waitKey(1);
//    cv::imshow("object location image",object_location_window);
//    cv::waitKey(1);
//    for(int i=0;i< vector_organized_back_projections.size();i++)
//    {
//        std::stringstream window_name;
//        window_name << "IMAGE " << i;
//        cv::imshow(window_name.str(),vector_organized_back_projections[i]);
//        cv::waitKey(1);
//    }

    // comment
//    cv::imshow("NEW OBJECT CENTROID WINDOW",new_object_centroid_window);
//    cv::waitKey(1);

    ptr_object_boundary_cloud_RGB->push_back(ptr_object_surface_cloud_RGB->at(index_seed_point)); // CENTROID
    ptr_object_boundary_cloud_RGB->push_back(ptr_object_surface_cloud_RGB->at(loc_min_x)); // LEFT
    ptr_object_boundary_cloud_RGB->push_back(ptr_object_surface_cloud_RGB->at(loc_min_y)); // TOP // BCZ loc_min_y is in image space, if a point is at top means its y is lower
    ptr_object_boundary_cloud_RGB->push_back(ptr_object_surface_cloud_RGB->at(loc_max_x)); // RIGHT
    ptr_object_boundary_cloud_RGB->push_back(ptr_object_surface_cloud_RGB->at(loc_max_y)); // BOTTOM

    std::cout << "POINTS PUSHED" << std::endl;
    //    std::cout << ptr_object_boundary_cloud_RGB->at(0) << std::endl  <<
    //                 ptr_object_boundary_cloud_RGB->at(1) << std::endl  <<
    //                 ptr_object_boundary_cloud_RGB->at(2) << std::endl  <<
    //                 ptr_object_boundary_cloud_RGB->at(3) << std::endl  <<
    //                 ptr_object_boundary_cloud_RGB->at(4) << std::endl;

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::copyPointCloud(*ptr_boundary_cloud_RGB,*ptr_boundary_cloud);

    //    std::cout << " PCL VIS" << std::endl;
    //    pcl::visualization::PCLVisualizer pcl_visualizer;
    //    pcl_visualizer.addCoordinateSystem(0.10,0.0,0.0,0.0);
    //    pcl_visualizer.addPointCloud(ptr_object_surface_cloud_RGB,"CLOUD 2");
    //    pcl_visualizer.addPointCloud(ptr_object_boundary_cloud_RGB,"CLOUD 3");

    //    int WHITE[]= {255,255,255};
    //    int GREEN[]= {0,255,0};
    //    int RED[]  = {255,0,0};
    //    int BLUE[] = {0,0,255};

    //    int* color_boundary_points[] = { WHITE,GREEN,RED,BLUE};

    //    for(int i=0;i<4;i++)
    //    {
    //        std::stringstream sphere_name;
    //        sphere_name << "SPHERE_" << i;
    //        pcl_visualizer.addSphere(ptr_object_boundary_cloud_RGB->at(i),0.02f,
    //                                 color_boundary_points[i][0], color_boundary_points[i][1],
    //                color_boundary_points[i][2],sphere_name.str());
    //    }

    //    pcl_visualizer.spinOnce(30000);
    //    pcl_visualizer.removeAllPointClouds();
    //    pcl_visualizer.removeAllShapes();
    //    pcl_visualizer.removeCoordinateSystem();
    //    pcl_visualizer.close();


    //    int no_of_valid_points =0;
    //    float total_weight =0.0;

    //    for(int i=0;i<ptr_bin_cloud_RGB->size();i++)
    //    {
    //        int min_x = object_location_rectangle.x;
    //        int min_y = object_location_rectangle.y;
    //        int max_x = object_location_rectangle.x + object_location_rectangle.width;
    //        int max_y = object_location_rectangle.y + object_location_rectangle.height;

    //        int point_x = ptr_pixel_cloud_RGB->at(i).x;
    //        int point_y = ptr_pixel_cloud_RGB->at(i).y;

    //        if(point_x > min_x && point_x < max_x && point_y > min_y && point_y < max_y )
    //        {
    //            float weight = final_posterior.at<float>(point_y,point_x);
    //            weight = 1.0f;

    //            centroid_3d.x += weight* ptr_bin_cloud_RGB->at(i).x;
    //            centroid_3d.y += weight* ptr_bin_cloud_RGB->at(i).y;
    //            centroid_3d.z += weight* ptr_bin_cloud_RGB->at(i).z;

    //            total_weight += weight;
    //            no_of_valid_points++;
    //        }
    //    }

    //    centroid_3d.x /= total_weight;
    //    centroid_3d.y /= total_weight;
    //    centroid_3d.z /= total_weight;
    //    centroid_3d.r =0;
    //    centroid_3d.g =255;
    //    centroid_3d.b =0;

    //    ptr_object_surface_cloud_RGB->push_back(centroid_3d);

    //    *ptr_bin_cloud_RGB += *ptr_object_surface_cloud_RGB;
    //    pcl::visualization::CloudViewer cloud_viewer("CLOUD");

    //    while(1)
    //    {
    //    cloud_viewer.showCloud(ptr_bin_cloud_RGB);sleep(1);
    //    cloud_viewer.showCloud(ptr_object_surface_cloud_RGB);sleep(1);

    //    }
    return true;
}


