#include <object_detection/objectDetect.h>
#include <object_detection/stowObjDetect.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


#include <object_detection/object_detection_helper.h>

#include <object_detection/rcnn_object_detection.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/imgproc/imgproc.hpp>

#include <tf/transform_listener.h>

#include <object_detection/rack_registration.h>

#include <object_detection/rcnn_object_detection_stowing.h>

cv::Mat hd_cam_frame;
bool hd_img_flag;
bool call_complete ;


using namespace std;
using namespace cv;

#define PTHREAD_FLAG 1
#define PT_CLOUD_SUBSCRIBE   0

void rgb_hd_img_callback(sensor_msgs::Image msg)
{
    cout << "Inside callback" << endl;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cout << "Trying to capture data" << endl;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    cout << "Before assigning camera data to hd_cam_frame " << endl;

    hd_cam_frame = cv::Mat(cv_ptr->image);

    cout << hd_cam_frame << endl;
    cout << "Camera img captured \t" << endl;

    if(hd_cam_frame.data != NULL)
        hd_img_flag = true;
    //    cv::imshow("rgb",rgb_img);
    //    cv::waitKey(1);

    call_complete = true;
    return;
}


class ObjectDetection{

private:

protected:
public:

    ros::Subscriber sub_ptCloud;
    ros::Subscriber sub_img;
    ros::ServiceServer detect_object;
    ros::ServiceClient rcnn_service;
    ros::ServiceServer rack_registration_service;
    ros::ServiceServer stow_detect_object;
    ros::ServiceClient rcnn_stow_task_service;
    ros::Publisher  pub_rcnn_image, pub_postpro_image;
    cv::Mat rgb_img;
#if(PT_CLOUD_SUBSCRIBE)
    bool capture_flag,capture_init_pc_flag;
#endif
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_new_scene_cloud_RGB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_old_scene_cloud_RGB;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_old_hough_points_cloud;

    ros::NodeHandle nh;

    // Bin resitration variables
    Eigen::Affine3f transformation_old_scene_to_model;


    ObjectDetection();
    ~ObjectDetection();
    bool detect_object_service_callback(object_detection::objectDetectRequest &req,
                                        object_detection::objectDetectResponse &res);
    bool stow_detect_object_service_callback(object_detection::stowObjDetectRequest &req,
                                             object_detection::stowObjDetectResponse &res);
    bool rack_registration_service_callback(object_detection::rack_registrationRequest &req,
                                            object_detection::rack_registrationResponse &res);
    //    void rgb_hd_img_callback(sensor_msgs::Image msg);
    void detect_object_callback(sensor_msgs::PointCloud2 msg);
    bool point2Dto3D(cv::Vec2i pixel, pcl::PointXYZ &point3d);

};



ObjectDetection::ObjectDetection()
{

#if(PT_CLOUD_SUBSCRIBE)
    capture_flag = false;
    capture_init_pc_flag = false;
#endif
    ptr_old_scene_cloud_RGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    ptr_new_scene_cloud_RGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pub_rcnn_image = nh.advertise<sensor_msgs::Image>("/object_detection/rcnn_recognition", 5);
    pub_postpro_image = nh.advertise<sensor_msgs::Image>("/object_detection/post_processed", 5);

    hd_img_flag = false;
    call_complete = false;

}

ObjectDetection::~ObjectDetection()
{

}

bool ObjectDetection::rack_registration_service_callback(object_detection::rack_registrationRequest &req,
                                                         object_detection::rack_registrationResponse &res)
{

    cout << "Inside rack registration service callback" << endl;

    vector<double> corner_point_vector;
    for(vector<double>::const_iterator it = req.corner_points.data.begin();it!=req.corner_points.data.end();++it)
        corner_point_vector.push_back(*it);

    cout << "Corner points \t" << endl;
    cout << cv::Mat(corner_point_vector) << endl;

    pcl::fromROSMsg(req.input_rgb_cloud,*ptr_old_scene_cloud_RGB);


#if(PT_CLOUD_SUBSCRIBE)
    capture_init_pc_flag = true;
    //    sleep(1);
    while(1){
        if(ptr_old_scene_cloud_RGB->empty())
        {
            ros::spinOnce();

        }
        else
        {
            capture_init_pc_flag = false;
            break;
        }

    }
#endif
    //    sleep(10);


    //-----GET TRANSFORMATION FROM SCENE TO MODEL

    cout << "Cloud size \t" << ptr_old_scene_cloud_RGB->size() << endl;

    SHELF_DETAILS shelf_deatils;
    get_shelf_details(shelf_deatils);

    // Store transformation matrix for a column
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_old_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ptr_old_hough_points_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*ptr_old_scene_cloud_RGB,*ptr_old_scene_cloud);

    for(int i=0;i<4;i++)
    {
        pcl::PointXYZ point;
        point.x =  corner_point_vector[i*3];
        point.y =  corner_point_vector[i*3+1];
        point.z =  corner_point_vector[i*3+2];
        ptr_old_hough_points_cloud->push_back(point);

    }

    std::string path = ros::package::getPath("object_detection").append("/image_database/shelf_models/bin_all.pcd");
    get_transformation_from_scene_to_model(shelf_deatils,ptr_old_scene_cloud,ptr_old_hough_points_cloud,transformation_old_scene_to_model,path);

    //------GOT THE TRANSFORMATION

    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_transformed_old_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    pcl::transformPointCloud(*ptr_old_scene_cloud_RGB,*ptr_transformed_old_scene_cloud_RGB,transformation_old_scene_to_model);

    //    pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
    //    while(1)
    //    {
    //      cloud_viewer.showCloud(ptr_old_scene_cloud_RGB);sleep(1);
    //      cloud_viewer.showCloud(ptr_transformed_old_scene_cloud_RGB);sleep(1);
    //    }

    ////    viewer.close();

    res.result.data = true;

    return true;
}

void getTransform(string src, string trg, tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;

    transform_listener.waitForTransform(trg, src, ros::Time(0),
                                        ros::Duration(2));
    transform_listener.lookupTransform(trg, src, ros::Time(0), transform);
    return;
}

void cropPixels(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, Mat
                &color_img, vector<geometry_msgs::Point> &bin_corners,
                vector<cv::Vec2i> &pixel_coordinates)
{
    string src_frame = "/camera_depth_optical_frame";
    string trg_frame =  "/wam_base_link";
    tf::StampedTransform transform;
    getTransform(src_frame, trg_frame, transform);

    vector<pcl::PointXYZ> c(4);

    for(int i = 0; i < bin_corners.size(); i++)
    {
        geometry_msgs::Point corners = bin_corners[i];
        c[i].x = corners.x;
        c[i].y = corners.y;
        c[i].z = corners.z;

        cout << "Corner " << i+1 << ": [" << c[i].x << " " << c[i].y
             << " " << c[i].z << "]" << endl;
    }


    if(cloud_ptr->empty())
    {

        cout << "Cloud is empty" << endl;
        return;
    }


    // Take each point from point cloud and transform it to wam base frame
    // Compare with the corner points and select those which are
    //within the corner points
    vector<int> indices;
    Mat b_img = Mat::zeros(color_img.rows, color_img.cols,
                           CV_8UC1);//(obj.kinect_image.size(),CV_8UC1,0);
    //    Mat color_img(cloud_ptr->height, cloud_ptr->width, CV_8UC3);
    //    cout << "Image size: [" << b_img.size() << "]" << endl;
    int px_min = b_img.rows, px_max = 0, py_min = b_img.cols, py_max = 0;
    for(int i = 0; i < b_img.rows; i++)
    {
        for(int j = 0; j < b_img.cols; j++)
        {
            pcl::PointXYZRGB pt = cloud_ptr->points[i*cloud_ptr->width+j];

            tf::Vector3 v(pt.x,pt.y,pt.z);
            tf::Vector3 tf_pt = transform*v;
            if(tf_pt.getX() <= c[0].x && tf_pt.getZ() <= c[0].z &&
                    tf_pt.getX() >= c[1].x && tf_pt.getZ() <= c[1].z &&
                    tf_pt.getX() >= c[2].x && tf_pt.getZ() >= c[2].z &&
                    tf_pt.getX() <= c[3].x && tf_pt.getZ() >= c[3].z)
            {
                b_img.at<uchar>(i,j) = 255;
                indices.push_back(i*b_img.cols+j);

                if(i<px_min)
                    px_min = i;
                else if(i>px_max)
                    px_max = i;
                if(j<py_min)
                    py_min = j;
                else if(j>py_max)
                    py_max = j;
                cout << "[" << px_min << ","  << py_min << "]" << endl;
                cout << "[" << px_max << ","  << py_max << "]" << endl;
            }
        }
    }

    cout << "[" << px_min << ","  << py_min << "]" << endl;
    cout << "[" << px_max << ","  << py_max << "]" << endl;

    cout << "Range cropping finished \n no. pts: " << indices.size() << endl;
    //    recropping(b_img, px_min, px_max, py_min, py_max, obj, transform, c);

    // Copy the pixels coordinates
    cv::Vec2i pixel(px_min,py_min);
    pixel_coordinates.push_back(pixel);

    pixel[0] = px_max;
    pixel[1] = py_max;

    // split the channels, do masking for each channel and recombine them
    vector<Mat> img_channels;
    vector<Mat> masked_channels(color_img.channels());
    split(color_img, img_channels);
    for(int i = 0; i < color_img.channels(); i++)
        bitwise_and(b_img,img_channels[i],masked_channels[i]);

    Mat cropped_img;
    merge(masked_channels,cropped_img);

    imshow("Bin_Mask",b_img);
    waitKey(1);
    imshow("Crop", cropped_img);
    waitKey(0);


    //    cout << "No. of indices: " << indices.size() << endl;
    // create the masked point cloud
    //    obj.cropped_pc_ptr->resize(indices.size());
    //    for(int i = 0; i < indices.size(); i++)
    //        obj.cropped_pc_ptr->points[i] =
    //    kinect_ptcloud_ptr->points[indices[i]];
    return;
}

//bool ObjectDetection::point2Dto3D(cv::Vec2i pixel, pcl::PointXYZ &point3d)
//{
//    int width = pcl_input_cloud->width, height = pcl_input_cloud->height;
//    int row = pixel[0], col = pixel[1];

//    for(int kernel=0; kernel<100; kernel++)
//    {
//        bool is_nan = true;
//        double sum_x = 0, sum_y = 0, sum_z = 0;
//        int count = 0;
//        for(int n=row-kernel; n<=row+kernel; n++)
//        {
//            for(int m=col-kernel; m<=col+kernel; m++)
//            {
//                if(n >=0 && n < height && m >= 0 && m < width)
//                {
//                    pcl::PointXYZRGB pt = pcl_input_cloud->at(n*width+m);
//                    if(!isnan(pt.x))
//                    {
//                        sum_x += pt.x;
//                        sum_y += pt.y;
//                        sum_z += pt.z;
//                        count++;
//                        is_nan = false;
//                    }
//                }
//            }
//        }
//        if(!is_nan)
//        {
//            point3d.x = sum_x/count;
//            point3d.y = sum_y/count;
//            point3d.z = sum_z/count;
//            return true;
//        }
//        else
//            return false;// could not get any valid point within a
//    }

//}

bool ObjectDetection::detect_object_service_callback(object_detection::objectDetectRequest &req,
                                                     object_detection::objectDetectResponse &res)
{
    //int p;
    //cout<<"Enter number"<<endl;
    //cin>>p;

    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    pcl::fromROSMsg(req.input_cloud,*pcl_input_cloud);

    //    cv_bridge::CvImagePtr cv_ptr;
    //    try
    //    {
    //        cv_ptr = cv_bridge::toCvCopy(req.input_rgb_img, sensor_msgs::image_encodings::BGR8);
    //    }
    //    catch (cv_bridge::Exception& e)
    //    {
    //        ROS_ERROR("cv_bridge exception: %s", e.what());
    //        return false;
    //    }

    //    cv::Mat img = cv::Mat(cv_ptr->image);

    //    rgb_img.release();

    pcl::fromROSMsg(req.input_rgb_cloud,*ptr_new_scene_cloud_RGB);


#if(PT_CLOUD_SUBSCRIBE)
    capture_flag = true;
    cout << req.bin_num << "\t" << req.bin_object_list.data.size() << endl;
    //    sleep(1);
    //    rgb_img.release();
    //    pcl_input_cloud->clear();
    while(1){
        if(rgb_img.data == NULL || ptr_new_scene_cloud_RGB->empty())
        {
            ros::spinOnce();

        }
        else
        {
            capture_flag = false;
            break;
        }
    }

    //    sleep(10);
#endif

    //------------COMPUTING TRANSFORMATION FROM CLOSER VIEW TO OLDER VIEW

    cout << "Saved transformation matrix \t" << endl;
    cout << transformation_old_scene_to_model.matrix() << endl;

    Eigen::Affine3f transformation_new_scene_to_old_scene;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_new_hough_points_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0;i<4;i++)
    {
        pcl::PointXYZ point;
        point.x =req.transformation_old_new.data[i*3];
        point.y =req.transformation_old_new.data[i*3+1];
        point.z =req.transformation_old_new.data[i*3+2];
        ptr_new_hough_points_cloud->push_back(point);

    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> transformation_estimator;
    transformation_estimator.estimateRigidTransformation(*ptr_new_hough_points_cloud,*ptr_old_hough_points_cloud,
                                                         transformation_new_scene_to_old_scene.matrix());

    cout << "Transofmration from new scene to old scene" << endl;
    cout << transformation_new_scene_to_old_scene.matrix() << endl;

    //---------------END COMPUTING TRANSFORMATIONS



    //    while(rgb_img.data == NULL && pcl_input_cloud->empty())
    //        ros::spinOnce();

    //    cv::VideoCapture cap(1);

    //    cap >> rgb_img;

    //    cap.release();


    //    vector<double> bin_corners_vector;
    //    vector<cv::Vec2i> pixelCoordinates;

    //    // Get bin corners from the service
    //    for(vector<double>::const_iterator it= req.bin_corners.data.begin();it != req.bin_corners.data.end();++it)
    //    {
    //        bin_corners_vector.push_back(*it);
    //    }

    //    cout << cv::Mat(bin_corners_vector) << endl;

    //    vector<geometry_msgs::Point> bin_corners(4);

    //    for(int i=0;i<4;i++)
    //    {
    //        bin_corners[i].x = bin_corners_vector[i*3];
    //        bin_corners[i].y = bin_corners_vector[(i*3)+1];
    //        bin_corners[i].z = bin_corners_vector[(i*3)+2];
    //    }


    //    cv::imshow("test",rgb_img);
    //    cv::waitKey(0);

    //    cropPixels(ptr_new_scene_cloud_RGB,rgb_img,bin_corners,pixelCoordinates);



    //    cv::Rect myrect = cv::Rect(pixelCoordinates[0][1],pixelCoordinates[0][0],pixelCoordinates[1][1]-pixelCoordinates[0][1]
    //            ,pixelCoordinates[1][0]-pixelCoordinates[0][0]);

    //    //    cv::Rect myrect = cv::Rect(150,243,237,160);
    //    rgb_img = rgb_img(myrect);


    //   cout << "Showing rgb image" << endl;


    //    cv::imshow("service_rgb_img",rgb_img);
    //    cv::waitKey(0);
    //    cv::destroyAllWindows();

    // send rgb image to RCNN and get possible window for the obj_id


    //------------UNWRAPPING BIN NUMBER AND BIN MEMBERS DATA FROM ROS MESSAGE

    int bin_num = req.bin_num.data;
    int total_objects = req.bin_object_list.data.size();

    std::vector<int> bin_object_list;

    for(std::vector<double>::const_iterator it = req.bin_object_list.data.begin(); it!=req.bin_object_list.data.end();++it)
        bin_object_list.push_back(*it);

    cout << "Object id \t" << bin_object_list[0] << endl;

    //------------UNWRAPPING DONE

    //----------FINE BIN CROPPING

    Eigen::Affine3f transformation_new_scene_to_model = Eigen::Affine3f::Identity();
    transformation_new_scene_to_model = transformation_old_scene_to_model * transformation_new_scene_to_old_scene;

    //    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed_new_hough_points_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::transformPointCloud(*ptr_new_hough_points_cloud,*ptr_transformed_new_hough_points_cloud,
    //                             transformation_new_scene_to_model);

    //    pcl::transformPointCloud(*ptr_new_hough_points_cloud_RGB,*ptr_new_hough_points_cloud_RGB,
    //                             transformation_new_scene_to_model);

    //    pcl::transformPointCloud(*ptr_old_hough_points_cloud,*ptr_old_hough_points_cloud,
    //                             transformation_old_scene_to_model);

    //    pcl::transformPointCloud(*ptr_new_scene_cloud_RGB,*ptr_new_scene_cloud_RGB,
    //                             transformation_new_scene_to_model);

    //    *ptr_new_scene_cloud_RGB += *ptr_new_hough_points_cloud_RGB;

    //    pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
    //    while(1)
    //         {
    //        cout << "new rgb cloud transformed" << endl;
    //        cloud_viewer.showCloud(ptr_new_scene_cloud_RGB);sleep(1);
    //          cout << "old points" << endl;
    //          cloud_viewer.showCloud(ptr_old_hough_points_cloud);sleep(1);
    //          cout << "transformed new  points" << endl;
    //          cloud_viewer.showCloud(ptr_transformed_new_hough_points_cloud);sleep(1);
    //          cout << " new  points" << endl;
    //          cloud_viewer.showCloud(ptr_new_hough_points_cloud);sleep(1);
    //         }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_pixel_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    construct_registred_image_RGB_cloud(ptr_new_scene_cloud_RGB,ptr_pixel_cloud_RGB);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_bin_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

    SHELF_DETAILS shelf_details;
    get_shelf_details(shelf_details);

    //std::cout << "ENTER BIN NUMBER" <<  std::endl;
    //std::cin >> bin_num;

    std::string database_path = ros::package::getPath("object_detection");
    std::string timestamp = std::to_string(ros::Time::now().toSec());


    remove_bin_boundaries(shelf_details,ptr_new_scene_cloud_RGB,ptr_pixel_cloud_RGB,
                          ptr_bin_cloud_RGB,bin_num,transformation_new_scene_to_model);

    pcl::transformPointCloud(*ptr_new_scene_cloud_RGB,*ptr_new_scene_cloud_RGB,transformation_new_scene_to_model);

    pcl::io::savePLYFileBinary(database_path + "/image_database/internal_images/ptr_new_scene_cloud_RGB" + timestamp + ".ply",*ptr_new_scene_cloud_RGB );
    pcl::io::savePLYFileBinary(database_path + "/image_database/internal_images/ptr_bin_cloud_RGB" + timestamp + ".ply", *ptr_bin_cloud_RGB);
    pcl::io::savePLYFileBinary(database_path + "/image_database/internal_images/ptr_pixel_cloud_RGB" + timestamp + ".ply",*ptr_pixel_cloud_RGB );

    cout << "BOUNDARIES REMOVED" << endl;

    //        pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
    //        while(1)
    //        {
    //        cloud_viewer.showCloud(ptr_bin_cloud_RGB);sleep(1);
    //        cloud_viewer.showCloud(ptr_new_scene_cloud_RGB);sleep(1);
    //        }

    //   -------GETTING CROPPED RECTANGLE FROM CROPPED BIN

    cv::Rect2i  rect_cropped_image;
    get_rectangle_for_cropping_image(ptr_pixel_cloud_RGB,rect_cropped_image);

    std::cout << "CROPPED RECTANGLE = " << rect_cropped_image << std::endl;

    if(rect_cropped_image.x < 0 || rect_cropped_image.y < 0 ||
            rect_cropped_image.width <=0 || rect_cropped_image.height <=0)
        return false;

    cv::Mat image_bgr;

    image_bgr.create(ptr_new_scene_cloud_RGB->height,ptr_new_scene_cloud_RGB->width,CV_8UC3);

    for(int i=0;i<image_bgr.rows;i++)
        for(int j=0;j<image_bgr.cols;j++)
        {
            unsigned char* pixel = (image_bgr.data+i* image_bgr.step[0]+j*image_bgr.step[1]);
            pixel[0] = ptr_new_scene_cloud_RGB->at(j,i).b;
            pixel[1] = ptr_new_scene_cloud_RGB->at(j,i).g;
            pixel[2] = ptr_new_scene_cloud_RGB->at(j,i).r;
        }

    cv::Mat rectangled_image;
    image_bgr.copyTo(rectangled_image);
    cv::rectangle(rectangled_image,rect_cropped_image,cv::Scalar(0,255,0),3);

    // comment
//    cv::imshow("recta",rectangled_image);
//    cv::waitKey(10);

    std::cout << "BIN NUMBER = " << bin_num <<  std::endl;

    //----- CROPPED RECTANGLE DONE

    //------ FINDING IMAGE FROM HD CAMERA



    cout << "BEfore video capturing \t" << endl;

    //    ros::NodeHandle n_img;
    //    ros::Subscriber sub = n_img.subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,rgb_hd_img_callback);

    //    cout << "After subscription \t" << endl;
    //    while(1)
    //    {
    //        if(hd_img_flag)
    //        {
    //            cout << "Camera data is not null" << endl;

    //            break;
    //        }
    //        else
    //        {
    //            if(call_complete== true)
    //            {
    //            cout << "Spinning img subscriber \t" << endl;
    //            ros::spinOnce();
    //            call_complete = false;
    //            }
    //        }
    //    }

    cout << "Before subscriber shutdown\t" << endl;

    //    sub.shutdown();


    cout << "After subscriber shutdown" << endl;
    //            obj_det->sub_img = obj_det->nh.subscribe<sensor_msgs::Image>("/usb/image_raw",1,&ObjectDetection::rgb_img_callback,obj_det);



    //    cv::VideoCapture cap(1);

    //    if(!cap.isOpened())
    //    {
    //        cout << "USB came device not opened \t" << endl;
    //        exit(0);
    //    }

    //    cv::Mat hd_cam_frame ;


    //    cap >> hd_cam_frame;

    //    while(1)
    //    {
    //        cap >> hd_cam_frame;
    //        if(hd_cam_frame.data != NULL)
    //        {
    //            cap.release();
    //            break;

    //        }
    //    }

    //    cout << "Captured frame \t" << endl;

    //    // Warp affine transformation
    //    float transformation_matrix[6];
    //    transformation_matrix[0] =  0.830729005826282f;
    //    transformation_matrix[1] = -0.04955427912626361f;
    //    transformation_matrix[2] = 60.02159636011777f;
    //    transformation_matrix[3] = -0.01105552467420169f;
    //    transformation_matrix[4] = 0.837069977147798f;
    //    transformation_matrix[5] = 129.447409054413f;

    //    cv::Mat transformation_hd_image_to_kinect_image(2,3,CV_32FC1);
    //    for(int i=0;i< transformation_hd_image_to_kinect_image.rows;i++)
    //        for(int j=0;j<transformation_hd_image_to_kinect_image.cols;j++)
    //            transformation_hd_image_to_kinect_image.at<float>(i,j) = transformation_matrix[i*3+j];

    //    cv::warpAffine(hd_cam_frame,hd_cam_frame,transformation_hd_image_to_kinect_image,hd_cam_frame.size());

    //    while(1)
    //    {
    //cv::imshow("KINECT IMAGE",image_bgr);
    //cv::imshow("transformed hd image",hd_cam_frame);
    //cv::imshow("Cropped Kinect image",image_bgr(rect_cropped_image));
    //cv::imshow("Cropped transformed image", hd_cam_frame(rect_cropped_image));
    //cv::waitKey(10);
    //    }

    // CONVERT RGB IMAGE TO ROS MESSGAE


    image_bgr.copyTo(hd_cam_frame);
    sensor_msgs::Image ros_img_msg;
    cv_bridge::CvImage img_bridge;

    int static counter = 0;

    std_msgs::Header header;
    header.seq = counter;

    header.stamp = ros::Time::now();

    img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,hd_cam_frame(rect_cropped_image));
    img_bridge.toImageMsg(ros_img_msg);

    // newly added to display color image on to rviz
    pub_rcnn_image.publish(ros_img_msg);


    counter++;
    //-----MESSAGE WRAPPING DONE


    //   cout << image_bgr

    //   std::vector<cv::Mat> vector_image;

    //   vector_image.push_back(image_bgr);
    //   vector_image.push_back(image_bgr(rect_cropped_image));

    //   pthread_t multi_t;

    //   pthread_create(&multi_t,NULL,multithread_imshow,(void *)&vector_image);

    //    std::thread multi_thrd_imshow(multithread_imshow,(std::vector<cv::Mat>&)vector_image);
    //    multi_thrd_imshow.detach();

    //    while(1)
    //    {
    //        cv::imshow("IMAGE",image_bgr);
    //        cv::imshow("IMAGE CROPPED", image_bgr(rect_cropped_image));
    //        cv::waitKey(10);
    //    }
    //----------

    //-----------CREATING RCNN REQUEST

    object_detection::rcnn_object_detection rcnn_req;
    // NOTE" Remeber the first element in the object list is always target object

    rcnn_req.request.obj_id.data = bin_object_list[0];
    rcnn_req.request.input_rgb_img = ros_img_msg;

    // Call RCNN service to get object window for obj_id


    std::vector<int> objwindowPoints;
    double objWindowScore = 0;

    if(rcnn_service.call(rcnn_req))
    {
        cout << "RCNN called successfully" << endl;

        for(std::vector<int>::const_iterator it = rcnn_req.response.obj_box_rect.data.begin();it!=rcnn_req.response.obj_box_rect.data.end()
            ;++it)
        {
            objwindowPoints.push_back(*it);
        }

        objWindowScore = rcnn_req.response.score.data;
    }
    else
    {
        cout << "Unable to call RCNN service" << endl;
    }
    //----------RCNN SERVICE DONE

    std::cout << "RCNN SCORE RETRIEVED" << std::endl;

    if(objwindowPoints.size() > 0)
    {
        cout << "Object details \t" << endl;
        cout << cv::Mat(objwindowPoints) << endl;
        cout << objWindowScore << endl;

        std::vector<cv::Rect2i> object_windows;
        std::vector<double> scores_windows;

        int win_x = objwindowPoints[0];
        int win_y = objwindowPoints[1];
        int width = objwindowPoints[2] - objwindowPoints[0]+1;
        int height = objwindowPoints[3] - objwindowPoints[1]+1;

        object_windows.push_back(cv::Rect2i(win_x,win_y,width,height));

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_normalized_pixel_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int i=0;i<ptr_pixel_cloud_RGB->size();i++)
        {
            pcl::PointXYZRGB point;
            point.x = ptr_pixel_cloud_RGB->at(i).x - rect_cropped_image.x;
            point.y = ptr_pixel_cloud_RGB->at(i).y - rect_cropped_image.y;
            point.z = ptr_pixel_cloud_RGB->at(i).z;
            point.r = ptr_pixel_cloud_RGB->at(i).r;
            point.g = ptr_pixel_cloud_RGB->at(i).g;
            point.b = ptr_pixel_cloud_RGB->at(i).b;
            ptr_normalized_pixel_cloud_RGB->push_back(point);
        }


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_object_surface_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_object_boundary_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::Normal centroid_normal;

        bool recognition_result = false;

        cv::Mat rcnn_img, post_pro_img;

        recognition_result =  recognize_object(object_windows,scores_windows,bin_object_list,image_bgr,rect_cropped_image,
                                               ptr_bin_cloud_RGB,ptr_normalized_pixel_cloud_RGB,ptr_object_surface_cloud_RGB,
                                               ptr_object_boundary_cloud_RGB,centroid_normal,database_path,timestamp,
                                               rcnn_img, post_pro_img);

        // newly added
        sensor_msgs::Image ros_img_msg1, ros_img_msg2;
        cv_bridge::CvImage img_bridge1, img_bridge2;

        std_msgs::Header header1, header2;
//        header1.seq = counter;

        header1.stamp = ros::Time::now();

        img_bridge1 = cv_bridge::CvImage(header1,sensor_msgs::image_encodings::BGR8,rcnn_img);
        img_bridge1.toImageMsg(ros_img_msg1);

        // newly added to display color image on to rviz
        pub_rcnn_image.publish(ros_img_msg1);

        header2.stamp = ros::Time::now();

        img_bridge2 = cv_bridge::CvImage(header2,sensor_msgs::image_encodings::BGR8,post_pro_img);
        img_bridge2.toImageMsg(ros_img_msg2);
        pub_postpro_image.publish(ros_img_msg2);

        if(!recognition_result)
            return false;

        pcl::transformPointCloud(*ptr_object_surface_cloud_RGB,*ptr_object_surface_cloud_RGB, transformation_new_scene_to_model.inverse());
        pcl::transformPointCloud(*ptr_object_boundary_cloud_RGB,*ptr_object_boundary_cloud_RGB, transformation_new_scene_to_model.inverse());


        //        std::cout << " PCL VIS" << std::endl;
        //        pcl::visualization::PCLVisualizer pcl_visualizer;
        //        pcl_visualizer.addCoordinateSystem(0.10,0.0,0.0,0.0);
        //        pcl_visualizer.addPointCloud(ptr_new_scene_cloud_RGB,"CLOUD 1");
        //        pcl_visualizer.addPointCloud(ptr_object_surface_cloud_RGB,"CLOUD 2");
        //        pcl_visualizer.addPointCloud(ptr_object_boundary_cloud_RGB,"CLOUD 3");

        //        int WHITE[]= {255,255,255};
        //        int GREEN[]= {0,255,0};
        //        int RED[]  = {255,0,0};
        //        int BLUE[] = {0,0,255};

        //        int* color_boundary_points[] = { WHITE,GREEN,RED,BLUE};

        //        for(int i=0;i<4;i++)
        //        {
        //            std::stringstream sphere_name;
        //            sphere_name << "SPHERE_" << i;
        //            pcl_visualizer.addSphere(ptr_object_boundary_cloud_RGB->at(i),0.02f,
        //                                     color_boundary_points[i][0], color_boundary_points[i][1],
        //                    color_boundary_points[i][2],sphere_name.str());
        //        }

        //        pcl_visualizer.spinOnce(30000);
        //        pcl_visualizer.removeAllPointClouds();
        //        pcl_visualizer.removeAllShapes();
        //        pcl_visualizer.removeCoordinateSystem();

        //        pcl_visualizer.close();


        pcl::PointXYZRGB& centroid_3d = ptr_object_boundary_cloud_RGB->at(0);
        std::cout << centroid_3d << std::endl;


        std::cout << "SENDING RESPONSE DATA" << std::endl;

        res.centroid.x = centroid_3d.x;
        res.centroid.y = centroid_3d.y;
        res.centroid.z = centroid_3d.z;
        std::cout << "SENDING CENTROID DATA" << std::endl;

        res.normal.x = centroid_normal.normal[0];
        res.normal.y = centroid_normal.normal[1];
        res.normal.z = centroid_normal.normal[2];
        std::cout << "SENDING NORMAL DATA" << std::endl;

        res.obj_left.x = ptr_object_boundary_cloud_RGB->at(1).x;
        res.obj_left.y = ptr_object_boundary_cloud_RGB->at(1).y;
        res.obj_left.z = ptr_object_boundary_cloud_RGB->at(1).z;
        std::cout << "SENDING LEFT DATA" << std::endl;

        res.obj_top.x = ptr_object_boundary_cloud_RGB->at(2).x;
        res.obj_top.y = ptr_object_boundary_cloud_RGB->at(2).y;
        res.obj_top.z = ptr_object_boundary_cloud_RGB->at(2).z;
        std::cout << "SENDING TOP DATA" << std::endl;

        res.obj_right.x = ptr_object_boundary_cloud_RGB->at(3).x;
        res.obj_right.y = ptr_object_boundary_cloud_RGB->at(3).y;
        res.obj_right.z = ptr_object_boundary_cloud_RGB->at(3).z;
        std::cout << "SENDING RIGHT DATA" << std::endl;

        std::cout << "RESPONSE DATA SENT" << std::endl;

        return true;

    }
    else
    {
        return false;

    }

    rgb_img.release();

    ptr_new_scene_cloud_RGB->clear();


    return true ;
}

bool ObjectDetection::stow_detect_object_service_callback(object_detection::stowObjDetectRequest &req,
                                                          object_detection::stowObjDetectResponse &res)
{
    // center kinect view position point cloud
    req.center_rgb_cloud;

    std::string database_path = ros::package::getPath("object_detection");
    std::string timestamp = std::to_string(ros::Time::now().toSec());

    std::stringstream cloud_file_name;
    cloud_file_name << database_path
                    << "/image_database/stow_internal_images/tote_cloud" << timestamp << ".ply";


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_tote_center_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_tote_left_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_tote_right_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(req.center_rgb_cloud,*ptr_tote_center_cloud_RGB);
    pcl::io::savePLYFileBinary(cloud_file_name.str(),*ptr_tote_center_cloud_RGB);

    for(int i=0; i<req.tote_object_list.data.size(); i++)
        int obj_id = req.tote_object_list.data[i];


    // Get RGB Image from the pointcloud and pass it to rcnn service

    cv::Mat pt_rgb_img;

    pt_rgb_img.create(ptr_tote_center_cloud_RGB->height,ptr_tote_center_cloud_RGB->width,CV_8UC3);

    for(int i=0;i<pt_rgb_img.rows;i++)
        for(int j=0;j<pt_rgb_img.cols;j++)
        {
            unsigned char* pixel = (pt_rgb_img.data+i* pt_rgb_img.step[0]+j*pt_rgb_img.step[1]);
            pixel[0] = ptr_tote_center_cloud_RGB->at(j,i).b;
            pixel[1] = ptr_tote_center_cloud_RGB->at(j,i).g;
            pixel[2] = ptr_tote_center_cloud_RGB->at(j,i).r;
        }

    cv::imshow("LIVE IMAGE", pt_rgb_img);
    cv::waitKey(0);

    //cv::VideoCapture cap(1);
    //cap >> pt_rgb_img;


    //    sensor_msgs::Image ros_img_msg;
    //    cv_bridge::CvImage img_bridge;

    //    static int counter = 0;

    //    std_msgs::Header header;
    //    header.seq = counter;
    //    header.stamp = ros::Time::now();
    //    img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,pt_rgb_img);
    //    img_bridge.toImageMsg(ros_img_msg);

    //    counter++;

    //    object_detection::rcnn_object_detection_stowing rcnn_stowing_msg;

    //    rcnn_stowing_msg.request.input_rgb_img = ros_img_msg;

    //    for(int i=0;i<req.tote_object_list.data.size();i++)
    //        rcnn_stowing_msg.request.stow_obj_list.data.push_back(req.tote_object_list.data[i]);

    //    //---------- Call RCNN service to get object windows and their scores --------------------


    //    // Vectors storing object Windows and their scores
    //    vector<cv::Rect> stow_obj_rect_array;
    //    vector<double> stow_obj_rect_scores;


    //    if(rcnn_stow_task_service.call(rcnn_stowing_msg))
    //    {
    //        cout << "Service called successfully \t" << endl;

    //        for(int i=0;i<rcnn_stowing_msg.response.obj_box_rect.data.size();i+=4)
    //        {
    //            cv::Rect temp_rect;
    //            temp_rect.x = rcnn_stowing_msg.response.obj_box_rect.data[i];
    //            temp_rect.y = rcnn_stowing_msg.response.obj_box_rect.data[i+1];
    //            temp_rect.width = rcnn_stowing_msg.response.obj_box_rect.data[i+2]-
    //                    rcnn_stowing_msg.response.obj_box_rect.data[i];
    //            temp_rect.height = rcnn_stowing_msg.response.obj_box_rect.data[i+3]-
    //                    rcnn_stowing_msg.response.obj_box_rect.data[i+1];

    //            stow_obj_rect_array.push_back(temp_rect);


    //        }

    //        stow_obj_rect_scores.assign(rcnn_stowing_msg.response.score.data.begin(),
    //                                    rcnn_stowing_msg.response.score.data.end());



    //    }
    //    else
    //    {
    //        cout << "Unable to call RCNN stowing service" << endl;
    //    }



    //    if(stow_obj_rect_array.size())
    //    {
    //        int max_score_location;
    //        float max_score = -10000.0;
    //        for(int i=0;i<stow_obj_rect_scores.size();i++)
    //        {
    //            if(max_score < stow_obj_rect_scores[i])
    //            {
    //                max_score = stow_obj_rect_scores[i];
    //                max_score_location = i;
    //            }

    //        }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_pixel_tote_center_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered_tote_center_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);



    std::cout << ptr_filtered_tote_center_cloud_RGB->height << "  " <<
                 ptr_filtered_tote_center_cloud_RGB->width << "  " <<
                 ptr_pixel_tote_center_cloud_RGB->height << "  " <<
                 ptr_pixel_tote_center_cloud_RGB->width << "  " << std::endl;



    //std::cout <<"MAX SCORE LOCATION=" <<  max_score_location << std::endl;

    std::cout << "CONSTRUCTING  POINT CLOUD" << std::endl;
    for(int i=0;i < ptr_tote_center_cloud_RGB->height;i++)
        for(int j=0;j < ptr_tote_center_cloud_RGB->width;j++)
        {
            ptr_filtered_tote_center_cloud_RGB->push_back(ptr_tote_center_cloud_RGB->at(j,i));

            pcl::PointXYZRGB point;
            point.x = j;
            point.y = i;
            point.z = 0;
            point.r = ptr_tote_center_cloud_RGB->at(j,i).r;
            point.g = ptr_tote_center_cloud_RGB->at(j,i).g;
            point.b = ptr_tote_center_cloud_RGB->at(j,i).b;

            ptr_pixel_tote_center_cloud_RGB->push_back(point);
        }


    //    for(int i=stow_obj_rect_array[max_score_location].y;i < stow_obj_rect_array[max_score_location].y + stow_obj_rect_array[max_score_location].height;i++)
    //        for(int j=stow_obj_rect_array[max_score_location].x;j < stow_obj_rect_array[max_score_location].x + stow_obj_rect_array[max_score_location].width;j++)
    //        {
    //            ptr_filtered_tote_center_cloud_RGB->push_back(ptr_tote_center_cloud_RGB->at(j,i));

    //            pcl::PointXYZRGB point;
    //            point.x = j;
    //            point.y = i;
    //            point.z = 0;
    //            point.r = ptr_tote_center_cloud_RGB->at(j,i).r;
    //            point.g = ptr_tote_center_cloud_RGB->at(j,i).g;
    //            point.b = ptr_tote_center_cloud_RGB->at(j,i).b;

    //            ptr_pixel_tote_center_cloud_RGB->push_back(point);
    //        }
    //    std::cout << "CONSTRUCTING POINT CLOUD DONE" << std::endl;

    float field_limits[6] =
    {
        -0.23f,0.23f,
        -0.18f,0.005f,
        0.05f,0.735f
    };

    std::cout << "FILTERING POINT CLOUD" << std::endl;

    filter_point_cloud(ptr_filtered_tote_center_cloud_RGB,
                       ptr_filtered_tote_center_cloud_RGB,
                       ptr_pixel_tote_center_cloud_RGB,
                       true,false,false,NULL,field_limits,0.0,4);

    std::cout << "FILTERING POINT CLOUD DONE " << std::endl;

    pcl::PointXYZRGB centroid_3d;
    Eigen::Matrix3f stow_eigen_vectors;
    pcl::Normal normal;
    bool is_stow_suction;
    int direction;
    int centroid_index_2d;


    select_stowing_surface(ptr_filtered_tote_center_cloud_RGB,
                           centroid_3d,stow_eigen_vectors,
                           normal,is_stow_suction,direction,centroid_index_2d);

    if(normal.normal[2] < 0 )
        normal.normal[2] = normal.normal[2];
    else
        normal.normal[2] = -normal.normal[2];

    std::cout << "NORMAL POINTS = " <<   normal.normal[0] << " " <<
                 normal.normal[1] << " " <<
                 normal.normal[2] << std::endl;


    //pcl::visualization::PCLVisualizer pcl_visualizer;

    //pcl_visualizer.addCoordinateSystem(0.10,0.0,0.0,0.0);
    //pcl_visualizer.addPointCloud(ptr_filtered_tote_center_cloud_RGB,"CLOUD1");
    //pcl_visualizer.addLine(centroid_3d,normal_3d_RGB,"LINE");

    //while(1)
    //pcl_visualizer.spinOnce(40);

    std::cout << "IS STOW  = " << is_stow_suction << std::endl;

    //        for(int i=0;i < ptr_filtered_tote_center_cloud_RGB->size();i++)
    //        {
    //            centroid_3d_RGB.x += ptr_filtered_tote_center_cloud_RGB->at(i).x;
    //            centroid_3d_RGB.y += ptr_filtered_tote_center_cloud_RGB->at(i).y;
    //            centroid_3d_RGB.z += ptr_filtered_tote_center_cloud_RGB->at(i).z;
    //        }

    //        centroid_3d_RGB.x /= ptr_filtered_tote_center_cloud_RGB->size();
    //        centroid_3d_RGB.y /= ptr_filtered_tote_center_cloud_RGB->size();
    //        centroid_3d_RGB.z /= ptr_filtered_tote_center_cloud_RGB->size();

    //        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    //        ptr_kd_tree->setInputCloud(ptr_filtered_tote_center_cloud_RGB);

    //        std::vector<int> vector_indices;
    //        std::vector<float> vector_distances;
    //        ptr_kd_tree->nearestKSearch(centroid_3d_RGB,1,vector_indices,vector_distances);

    //        int index_seed_point;
    //        index_seed_point = vector_indices[0];

    //        centroid_3d_RGB.x = ptr_filtered_tote_center_cloud_RGB->at(index_seed_point).x;
    //        centroid_3d_RGB.y = ptr_filtered_tote_center_cloud_RGB->at(index_seed_point).y;
    //        centroid_3d_RGB.z = ptr_filtered_tote_center_cloud_RGB->at(index_seed_point).z;

    cv::Point2f centroid_2d;
    centroid_2d.x = ptr_pixel_tote_center_cloud_RGB->at(centroid_index_2d).x;
    centroid_2d.y = ptr_pixel_tote_center_cloud_RGB->at(centroid_index_2d).y;


    cv::Mat centroid_image;
    pt_rgb_img.copyTo(centroid_image);

    cv::circle(centroid_image,centroid_2d,7,cv::Scalar(0,255,0),-1);
    cv::imshow("CENTROID IMAGE",centroid_image);
    cv::waitKey(0);


    res.gripping_centroid.x = centroid_3d.x;
    res.gripping_centroid.y = centroid_3d.y;
    res.gripping_centroid.z = centroid_3d.z;

    res.gripping_normal.x = normal.normal[0];
    res.gripping_normal.y = normal.normal[1];
    res.gripping_normal.z = normal.normal[2];

    res.gripping_axis.x = stow_eigen_vectors(0,0);
    res.gripping_axis.y = stow_eigen_vectors(1,0);
    res.gripping_axis.z = stow_eigen_vectors(2,0);

    res.obj_centroid.x = centroid_3d.x;
    res.obj_centroid.y = centroid_3d.y;
    res.obj_centroid.z = centroid_3d.z;

    res.obj_normal.x = normal.normal[0];
    res.obj_normal.y = normal.normal[1];
    res.obj_normal.z = normal.normal[2];

    res.obj_axis.x = stow_eigen_vectors(0,0);
    res.obj_axis.y = stow_eigen_vectors(1,0);
    res.obj_axis.z = stow_eigen_vectors(2,0);

    res.flag_grip.data =  !is_stow_suction;
    res.obj_psn_suction.data = 5;

    std::cout << "DATA SENT" << std::endl;

    return true;
    //}
    //else
    //return false;
}


void ObjectDetection::detect_object_callback(sensor_msgs::PointCloud2 msg)
{

#if(PT_CLOUD_SUBSCRIBE)
    if(capture_flag)
    {
        //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(msg,*ptr_new_scene_cloud_RGB);


        //        cv::Mat pc_rgb_img = cv::Mat::zeros(pcl_input_cloud->height,pcl_input_cloud->width,CV_8UC3);
        rgb_img = cv::Mat::zeros(ptr_new_scene_cloud_RGB->height,ptr_new_scene_cloud_RGB->width,CV_8UC3);

        //    const float bad_point = std::numeric_limits<float>::quiet_NaN();

        for(int i=0; i<ptr_new_scene_cloud_RGB->height; i++)
            for(int j=0; j<ptr_new_scene_cloud_RGB->width; j++)
            {
                pcl::PointXYZRGB pt = ptr_new_scene_cloud_RGB->at(j,i);


                Eigen::Vector3i v = pt.getRGBVector3i();
                cv::Vec3b color(v(2),v(1),v(0));
                rgb_img.at<cv::Vec3b>(i,j) = color;

            }

        //    cv::imshow("pc_rgb_img",rgb_img);
        //    cv::waitKey(1);

    }

    //    cout << "Catprue rack flag" << "\t" << capture_init_pc_flag << endl;

    if(capture_init_pc_flag)
    {
        //        cout << "Capturing point cloud for rack registration" << endl;
        pcl::fromROSMsg(msg,*ptr_old_scene_cloud_RGB);



    }


#endif
    return ;
}


