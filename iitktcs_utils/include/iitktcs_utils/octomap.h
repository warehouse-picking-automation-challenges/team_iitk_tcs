#ifndef OCTOMAP_H
#define OCTOMAP_H

#include<iostream>
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
#define KEY_COLOR (int)1048686
#define POlYGON_ANNOTATE 1
#define SHOW_ARROW 1

using namespace std;
using namespace cv;
using namespace pcl;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;


vector<int > roi;

typedef struct _MouseData{
    int event;
    int flag;
    cv::Point2i point;
    bool data_valid;
}MouseData;

void mouse_callback_handler(int event, int x, int y, int flag, void* userdata);

void left_button_handler(MouseData& mouse_data,int color, std::vector<int>& polygons_color,int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                         cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons);

void right_button_handler(MouseData& mouse_data, std::vector<int>& polygons_color,int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                          cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons);

void middle_button_handler(cv::Mat& mask, std::vector<int>& polygons_color, std::vector<std::vector<cv::Point2i> >& polygons);

void mouse_movement_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                            cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons);

void get_mask(cv::Mat& ip_image,cv::Mat& mask);

void get_mask(cv::Mat& ip_image, cv::Mat &mask)
{

    cv::Mat loc_image[3];

    ip_image.copyTo(loc_image[0]);
    ip_image.copyTo(loc_image[1]);
    ip_image.copyTo(loc_image[2]);

    cv::merge(loc_image,3,ip_image);


    mask = cv::Mat::zeros(ip_image.rows, ip_image.cols, CV_8UC3);
    MouseData mouse_data;
    cv::MouseCallback mouse_callback = mouse_callback_handler;

    std::stringstream im_window_name;
    cv::namedWindow("image",CV_WINDOW_NORMAL);
    cv::moveWindow("image",0,0);

    cv::setMouseCallback("image", mouse_callback, &mouse_data);

    cv::Mat original_image;

    ip_image.copyTo(original_image);

    cv::Size original_image_size(original_image.cols, original_image.rows);

    float factor = 1.0f;

    cv::Mat image;
    cv::resize(original_image, image, cv::Size(original_image_size.width*factor,original_image_size.height*factor));
    //            original_image.copyTo(image);

    cv::Mat display_image;
    image.copyTo(display_image);

    std::vector<std::vector<cv::Point2i> > polygons;
    std::vector<int> polygons_color;

    mouse_data.data_valid = false;

    bool continue_with_same_image = true;
    bool single_polygon_completed = true;
    bool is_seed_point_available = false;
    int polygon_no = -1;


    while (continue_with_same_image)
    {

        int color = 255;

        while (!mouse_data.data_valid)
        {
            cv::imshow("image", display_image);
            cv::waitKey(2);
        }

        switch (mouse_data.event)
        {
        case cv::EVENT_LBUTTONDOWN:
            image.copyTo(display_image);
            left_button_handler(mouse_data,color,polygons_color, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
            break;

        case cv::EVENT_RBUTTONDOWN:
            image.copyTo(display_image);
            right_button_handler(mouse_data,polygons_color, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
            break;
        case cv::EVENT_MBUTTONDOWN:
            middle_button_handler(mask,polygons_color, polygons);

            cv::resize(mask, mask, original_image_size,CV_INTER_NN);
            continue_with_same_image = false;
            break;
        case cv::EVENT_MOUSEMOVE:
            image.copyTo(display_image);
            mouse_movement_handler(mouse_data, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
            break;
        }

        mouse_data.data_valid = false;
    }

    cv::destroyWindow("image");

    cv::cvtColor(mask,mask,CV_BGR2GRAY);
}

void left_button_handler(MouseData& mouse_data, int polygon_color, std::vector<int>& polygons_color, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                         cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons)
{
    if (single_polygon_completed)
    {
        std::vector<cv::Point2i> vector_point;
        polygons.push_back(vector_point);
        polygon_number++;
        single_polygon_completed = false;
        is_seed_point_available = false;
    }

    {
        cv::Scalar color(0, 255, 0);
        int thickness = 2;
        cv::Scalar color_circle(0, 0, 255);
        int point_circle_radii = 3;
        int circle_thickness = -1;

        {
            if (is_seed_point_available)
            {
                float polygon_termination_radii = 7.0;
                float x = polygons[polygon_number][0].x - mouse_data.point.x;
                float y = polygons[polygon_number][0].y - mouse_data.point.y;

                float distance = sqrtf(x*x + y*y);

                if (distance < polygon_termination_radii)
                {
                    single_polygon_completed = true;
                    printf("Polygon No %d Completed\n", polygon_number);
                }
                else
                {
                    polygons[polygon_number].push_back(mouse_data.point);
                }
            }
            else
            {
                //                int color;

                //                for(int i=0;i<10;i++)
                //                {
                //                    int a = cv::waitKey(1);
                //                    if(a >= 0)
                //                        color = a;
                //                }

                //                //                std::cout << "COLOR = " << (int)color << "  " << KEY_COLOR << "\n";

                //                if(color == KEY_COLOR)
                //                    color = 0;
                //                else
                //                    color = 255;

                std::cout << "COLOR = " << polygon_color << "\n";

                polygons_color.push_back(polygon_color);

                //                std::stringstream str_color;
                //                str_color << color;

                //                CvFont cv_font;
                //                cv_font.color = cv::Scalar(0,255,0);
                //                cv_font.thickness = 2;


                //                cv::addText(display_image, str_color.str(),cv::Point2i(0,0),cv_font);

                polygons[polygon_number].push_back(mouse_data.point);
                cv::circle(display_image, polygons[polygon_number][0], point_circle_radii, color_circle, circle_thickness);
                is_seed_point_available = true;
            }
        }



        if (single_polygon_completed)
        {
            for (int i = 0; i < polygons.size(); i++)
            {
                int polygon_size = polygons[i].size();
                for (int j = 0; j < polygon_size; j++)
                {
                    cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                    cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
                }
            }
        }
        else
        {
            for (int i = 0; i < polygons.size() - 1; i++)
            {
                int polygon_size = polygons[i].size();
                for (int j = 0; j < polygon_size; j++)
                {
                    cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                    cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
                }
            }

            int polygon_size = polygons[polygon_number].size();
            for (int j = 0; j < polygon_size - 1; j++)
            {
                cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
                cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
            }
            cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);

        }

    }
}

void right_button_handler(MouseData& mouse_data, std::vector<int>& polygons_color, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                          cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons)
{
    if (polygons.size())
    {
        if (polygons[polygon_number].size())
        {
            polygons[polygon_number].pop_back();
            single_polygon_completed = false;


        }


        if (!polygons[polygon_number].size())
        {
            polygons.pop_back();
            single_polygon_completed = true;
            polygon_number--;
            if (polygon_number > -1)
                is_seed_point_available = true;

            if(polygons_color.size())
                polygons_color.pop_back();

        }

        cv::Scalar color(0, 255, 0);
        int thickness = 2;
        cv::Scalar color_circle(0, 0, 255);
        int point_circle_radii = 3;
        int circle_thickness = -1;

        if (single_polygon_completed)
        {
            for (int i = 0; i < polygons.size(); i++)
            {
                int polygon_size = polygons[i].size();
                for (int j = 0; j < polygon_size; j++)
                {
                    cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                    cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
                }
            }
        }
        else
        {
            for (int i = 0; i < polygons.size() - 1; i++)
            {
                int polygon_size = polygons[i].size();
                for (int j = 0; j < polygon_size; j++)
                {
                    cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                    cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
                }
            }

            int polygon_size = polygons[polygon_number].size();
            for (int j = 0; j < polygon_size - 1; j++)
            {
                cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
                cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
            }
            cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);

        }

    }
}

void middle_button_handler(cv::Mat& mask, std::vector<int>& polygons_color, std::vector<std::vector<cv::Point2i> >& polygons)
{

    for(int i=0;i<polygons.size();i++)
    {
        std::vector<std::vector<cv::Point2i> > polygon;

        polygon.push_back(std::vector<cv::Point2i>());

        for(int j=0;j<polygons[i].size();j++)
        {             polygon[0].push_back(polygons[i][j]);

            //            std::cout << polygons[i][j].x << " " <<  polygons[i][j].y <<std::endl;
        }
        int polygon_color = polygons_color[i];
        cv::fillPoly(mask, polygon, cv::Scalar(polygon_color, polygon_color, polygon_color));
    }

    //        cv::fillPoly(mask, polygons, cv::Scalar(255, 255, 255));

}

void mouse_movement_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
                            cv::Mat& display_image, std::vector<std::vector<cv::Point2i> >& polygons)
{

    cv::Scalar color(0, 255, 0);
    int thickness = 2;
    cv::Scalar color_circle(0, 0, 255);
    int point_circle_radii = 3;
    int circle_thickness = -1;

    if (single_polygon_completed)
    {
        for (int i = 0; i < polygons.size(); i++)
        {
            int polygon_size = polygons[i].size();
            for (int j = 0; j < polygon_size; j++)
            {
                cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
            }
        }
    }
    else
    {
        for (int i = 0; i < polygons.size() - 1; i++)
        {
            int polygon_size = polygons[i].size();
            for (int j = 0; j < polygon_size; j++)
            {
                cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
                cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
            }
        }

        int polygon_size = polygons[polygon_number].size();
        for (int j = 0; j < polygon_size - 1; j++)
        {
            cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
            cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
        }
        cv::line(display_image, polygons[polygon_number][polygon_size - 1], mouse_data.point, color, thickness);
        cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);
        cv::circle(display_image, mouse_data.point, point_circle_radii, color_circle, circle_thickness);

    }
}

void mouse_callback_handler(int event, int x, int y, int flag, void* userdata)
{
    MouseData *mouse_data = (MouseData*)userdata;
    mouse_data->event = event;
    mouse_data->point.x = x;
    mouse_data->point.y = y;
    mouse_data->flag = flag;
    mouse_data->data_valid = true;
}

void extractPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,pcl::PointCloud<pcl::PointXYZ>::Ptr &extracted_scene,cv::Mat &image)
{

    cv::Mat mask;

    get_mask(image,mask);

    cv::imshow("mask", mask);
    cv::waitKey(0);


    pcl::PointIndices::Ptr in(new pcl::PointIndices);

    for(int row=0;row<mask.rows;row++)
        for(int col=0;col<mask.cols;col++)
        {
            if(mask.at<uchar>(row,col)==255)
            {
                in->indices.push_back(row*scene->width+col);
            }

        }
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (scene);
    extract.setIndices (in);
    extract.setNegative (false);
    extract.filter (*extracted_scene);

//    pcl::visualization::PCLVisualizer viewer1("cloud");
//    viewer1.addPointCloud(extracted_scene);
//    viewer1.spin();
//    viewer1.close();





}



void createRackImageCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat &ref_image)
{
    ref_image.release();
    ref_image.create(cloud->height,cloud->width,CV_8UC1);

    for(int row = 0;row<cloud->height ; row++)
        for(int col=0; col<cloud->width; col++)
        {

            pcl::PointXYZ pt = cloud->at(col,row);
            if(!isFinite(pt)&& pt.z > 0.05 && pt.z < 1.5)
                ref_image.at<uchar>(row,col) = 0;
            else
                ref_image.at<uchar>(row,col) = (pt.z/1.5)*255;
        }
    return;
}








#endif // OCTOMAP_H
