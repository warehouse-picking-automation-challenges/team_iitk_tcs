#include <iostream>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <iitktcs_msgs_srvs/GetFK.h>
#include <iitktcs_msgs_srvs/GetCloud.h>
#include <iitktcs_msgs_srvs/fetch_foscam_ensenso_cloud.h>

using namespace std;
using namespace cv;
using namespace pcl;

#define NO_JOINTS 6

#define NO_POINTS 12

// 0: to record values now and then compute calibrated matrix
// 1: to compute calibrated matrix from stored values
#define USE_STORED_VALUES 0

typedef pcl::PointXYZRGBA Point_Type;

Vec2i pixel_coord;
bool clicked = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if( event == EVENT_LBUTTONDOWN && !clicked)
    {
        cout << "Left button clicked (" << y << ", " << x << ")" << endl;
        pixel_coord(0) = y;
        pixel_coord(1) = x;
        clicked = true;
    }
    return ;
}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<Point_Type>::Ptr &cloud_ptr)
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

bool point2Dto3D(Vec2i pixel, PointXYZ &point3d, pcl::PointCloud<Point_Type>::Ptr &cloud_ptr)
{
    int width = cloud_ptr->width, height = cloud_ptr->height;
    int row = pixel[0], col = pixel[1];

    for(int kernel=0; kernel<50; kernel++)
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
                    Point_Type& pt = cloud_ptr->at(n*width+m);
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

void getRobotPts(vector<PointXYZ> &record_pts, ros::ServiceClient &fk_service)
{
    iitktcs_msgs_srvs::GetFK getFK;

    for(int i=0; i<NO_POINTS; i++)
    {
        cout << "Place robot suction end effector tip at marker: " << i+1 << endl;
        cout << "Press enter ..." << endl;
        getchar();

        getFK.request.current_jts.data = true;
        getFK.request.end_effector_link.data = "s_st_eef_link";
//        getFK.request.end_effector_link.data = "nozele";
        if(fk_service.call(getFK))
        {
//            cout << "Position: " << getFK.response.pose.position.x << " " << getFK.response.pose.position.y << " " << getFK.response.pose.position.z << endl;
//            cout << "Orientation: " << getFK.response.pose.orientation.x << " " << getFK.response.pose.orientation.y << " " << getFK.response.pose.orientation.z << " " << getFK.response.pose.orientation.w << endl;

//            vector<tf::Vector3> tf_points;
//            tf::Vector3 pt(desired_pose.position.x, desired_pose.position.y, desired_pose.position.z);

//            tf_points.push_back(pt);
//            sphereMarker(tf_points, sphere_markerarray, "/world");

            PointXYZ marker_rbt;
            marker_rbt.x = getFK.response.pose.position.x;
            marker_rbt.y = getFK.response.pose.position.y;
            marker_rbt.z = getFK.response.pose.position.z;
            cout << "{" << marker_rbt.x << ", " << marker_rbt.y << ", "<< marker_rbt.z << "}" << endl;

            record_pts.push_back(marker_rbt);
        }
        else
        {
            cout << "Failed to find forward kinematics" << endl;
        }

    }
    cout << "Recording marker position wrt to robot base link done" << endl << endl;

    return;
}


void get3DSensorPts(vector<PointXYZ> &record_pts, string cloud_topic, string image_topic, string window,
                    pcl::PointCloud<Point_Type>::Ptr src_cloud, cv::Mat src_image)
{
//    pcl::PointCloud<Point_Type>::Ptr src_cloud(new pcl::PointCloud<Point_Type> ());
//    cv::Mat src_image;
//    ros::NodeHandle node;
//    ros::Subscriber subimage = node.subscribe<sensor_msgs::Image>
//            (image_topic,1,boost::bind(imageCallback,_1,boost::ref(src_image)));
//    ros::Subscriber sub_cloud = node.subscribe<sensor_msgs::PointCloud2>
//            (cloud_topic, 1, boost::bind(kinectPCCallback, _1, boost::ref(src_cloud)));

    cv::namedWindow(window, cv::WINDOW_NORMAL);
    setMouseCallback(window, CallBackFunc, NULL);

    cout << "Subscribed to topics: \n" << image_topic << " " << cloud_topic << endl;
    int no_kntpts_recorded = 0;
    cout << "Place the arm at a position from where we can see all the markers" << endl;
    cout << "Do not change Arm's position now ..." << endl;
    cout << "Left click on the marker point" << endl;
    cout << "Caution: CLICK ON MARKER POINT IN SAME ORDER AS WHILE RECORDING WITH SUCTION TIP" << endl;
    while(ros::ok() && no_kntpts_recorded < NO_POINTS)
    {
//        ros::spinOnce();
        char c;
        if(!src_image.empty() && !src_cloud->empty())
        {
            cv::imshow(window, src_image);
            c = waitKey(1);
        }
        if(!src_cloud->empty() && clicked)
        {
            PointXYZ pt3d;
            if(point2Dto3D(pixel_coord, pt3d, src_cloud))
            {
                record_pts.push_back(pt3d);
                no_kntpts_recorded++;
                cout << "Sensor pt " << no_kntpts_recorded << "\n{" << pt3d.x << ", " << pt3d.y << ", " << pt3d.z << "}" << endl;
                clicked = false;
            }
        }

        if(c == 'q' || c==27)// if q or Esc is press over image window then calibrate the transformation between kinect and robot
            break;
    }
    cv::destroyWindow(window);
    return;
}

void addMarker(double *centroid, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
//    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

//    id++;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"calibrate");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    cout << "Will record " << NO_POINTS << endl;
    ros::Publisher pub_marker = node.advertise<visualization_msgs::MarkerArray>("/plot", 10);

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker sphere_marker;

    ros::ServiceClient dynamic_fk_service = node.serviceClient<iitktcs_msgs_srvs::GetFK>("/iitktcs/motion_planner/forward_kinematics");
    vector<PointXYZ> kinect_pts, robot_pts;
    pcl::PointCloud<pcl::PointXYZ> knt, rbt;

    // Declare the service for ensenso to publish point cloud
    // old service to capture cloud and image. No more used
//    ros::ServiceClient ensenso_pubcloud_serviceclient =
//            node.serviceClient<iitktcs_msgs_srvs::GetCloud>("/iitktcs/ensenso/get_cloud");

    // New service from ashish for publishing cloud and image from ensenso
    ros::ServiceClient ensenso_pubcloud_serviceclient =
            node.serviceClient<iitktcs_msgs_srvs::GetCloud>("/iitktcs/ensenso/registered_cloud");
    iitktcs_msgs_srvs::GetCloud ensenso_pub_cloud;
    ensenso_pub_cloud.request.get_cloud.data = true;

    // In case of sudden drop in recording of points wrt camera
//    double r_pts[][3] = {{0.927092, 0.269567, 0.537213},
//                         {0.924371, 0.127145, 0.532405},
//                         {0.907592, 0.021667, 0.533221},
//                         {0.909689, -0.0792896, 0.532421},
//                         {0.816533, 0.274621, 0.531797},
//                         {0.803717, 0.137878, 0.528489},
//                         {0.803426, 0.0288101, 0.529405},
//                         {0.791932, -0.0902796, 0.529033},
//                         {0.720247, 0.285258, 0.533122},
//                         {0.716177, 0.15458, 0.533288},
//                         {0.711521, 0.0453245, 0.530064},
//                         {0.703893, -0.0883924, 0.528939}
//                        };
//    for(int i=0; i<NO_POINTS; i++)
//    {
//        PointXYZ pt;
//        pt.x = r_pts[i][0]; pt.y = r_pts[i][1]; pt.z = r_pts[i][2];
//        robot_pts.push_back(pt);
//    }

#if(USE_STORED_VALUES)

    double r_pts[][3] = {{-1.00000,  -1.00000,   0.50000},
                         {-1.00000,  -0.80000,   0.50000},
                         {-1.00000,  -0.60000,   0.50000},
                         {-1.00000,  -0.40000,   0.50000},
                         {-1.00000,  -0.20000,   0.50000},
                         {-0.80000,  -1.00000,   0.50000},
                         {-0.80000,  -0.80000,   0.50000},
                         {-0.80000,  -0.60000,   0.50000},
                         {-0.80000,  -0.40000,   0.50000},
                         {-0.80000,  -0.20000,   0.50000},
                         {-0.60000,  -1.00000,   0.50000},
                         {-0.60000,  -0.80000,   0.50000},
                         {-0.60000,  -0.60000,   0.50000},
                         {-0.60000,  -0.40000,   0.50000},
                         {-0.60000,  -0.20000,   0.50000},
                         {-0.40000,  -1.00000,   0.50000},
                         {-0.40000,  -0.80000,   0.50000},
                         {-0.40000,  -0.60000,   0.50000},
                         {-0.40000,  -0.40000,   0.50000},
                         {-0.40000,  -0.20000,   0.50000},
                         {-0.20000,  -1.00000,   0.50000},
                         {-0.20000,  -0.80000,   0.50000},
                         {-0.20000,  -0.60000,   0.50000},
                         {-0.20000,  -0.40000,   0.50000},
                         {-0.20000,  -0.20000,   0.50000},
                        };
    for(int i=0; i<NO_POINTS; i++)
    {
        PointXYZ pt;
        pt.x = r_pts[i][0]; pt.y = r_pts[i][1]; pt.z = r_pts[i][2];
        robot_pts.push_back(pt);
    }

    double k_pts[][3] = {{-0.9624875,  -1.0985368,  -0.3969472},
                         {-0.9604684,  -0.9564439,  -0.2562151},
                         {-0.9584492,  -0.8143509,  -0.1154830},
                         {-0.9564301,  -0.6722579,   0.0252491},
                         {-0.9544110,  -0.5301650,   0.1659812},
                         {-0.7632112,  -1.0880892,  -0.4103550},
                         {-0.7611921,  -0.9459962,  -0.2696228},
                         {-0.7591730,  -0.8039032,  -0.1288907},
                         {-0.7571539,  -0.6618103,   0.0118414},
                         {-0.7551348,  -0.5197173,   0.1525735},
                         {-0.5639350,  -1.0776415,  -0.4237627},
                         {-0.5619159,  -0.9355486,  -0.2830306},
                         {-0.5598968,  -0.7934556,  -0.1422985},
                         {-0.5578777,  -0.6513626,  -0.0015664},
                         {-0.5558586,  -0.5092697,   0.1391657},
                         {-0.3646588,  -1.0671939,  -0.4371705},
                         {-0.3626396,  -0.9251009,  -0.2964384},
                         {-0.3606205,  -0.7830080,  -0.1557063},
                         {-0.3586014,  -0.6409150,  -0.0149742},
                         {-0.3565823,  -0.4988220,   0.1257580},
                         {-0.1653825,  -1.0567462,  -0.4505782},
                         {-0.1633634,  -0.9146533,  -0.3098461},
                         {-0.1613443,  -0.7725603,  -0.1691140},
                         {-0.1593252,  -0.6304673,  -0.0283819},
                         {-0.1573061,  -0.4883744,   0.1123502}
                        };
    for(int i=0; i<NO_POINTS; i++)
    {
        PointXYZ pt;
        pt.x = k_pts[i][0]; pt.y = k_pts[i][1]; pt.z = k_pts[i][2];
        kinect_pts.push_back(pt);
    }

#else

    getRobotPts(robot_pts, dynamic_fk_service);

    string pt_cloud_topic = "/iitktcs/ensenso/depth_registered/points";
    string img_topic = "/iitktcs/foscam/image_color";

    bool got_cloud = false;
    pcl::PointCloud<Point_Type>::Ptr src_cloud(new pcl::PointCloud<Point_Type>);
    cv::Mat image;
//    ros::Subscriber subimage = node.subscribe<sensor_msgs::Image>
//            (img_topic,1,boost::bind(imageCallback,_1,boost::ref(image)));
//    bool stop = false;
    cv::namedWindow("Calibrated_foscam_image", cv::WINDOW_NORMAL);
//    int count = 0;
    cout << "Capturing cloud and image from ensenso" << endl;
//    ros::Rate loop(30);
    while(ros::ok() && !got_cloud)
    {
        if(ensenso_pubcloud_serviceclient.call(ensenso_pub_cloud))
        {
            cout << ensenso_pub_cloud.response.sensor_cloud.width << " " << ensenso_pub_cloud.response.sensor_cloud.height << endl;
            pcl::fromROSMsg(ensenso_pub_cloud.response.sensor_cloud, *src_cloud);

            cout << src_cloud->width << " " << src_cloud->height << endl;
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
            viewer -> addCoordinateSystem(.1);
            viewer -> setBackgroundColor(0, 0, 0);
            viewer -> initCameraParameters();
            viewer -> addPointCloud(src_cloud);
            while(!viewer -> wasStopped())
                viewer -> spinOnce();

            createImage(src_cloud, image);

//            count++;
//            cout << "Count: " << count << endl;
//            while(ros::ok() && image.empty())
//            {
//                cout << "Waiting for image from topic: " << img_topic << endl;
//                loop.sleep();
//                ros::spinOnce();
//            }

            cout << "***** Press 'c': start capturing camera points" << endl;
            cout << "***** Press 'ESC': for next camera view" << endl;
            while(ros::ok() && !image.empty())
            {
                cv::imshow("Calibrated_foscam_image", image);

                char c = cv::waitKey(1);
                if(c=='c')
                {
                    got_cloud = true;
                    break;
                }
                else if(c==27)
                    break;
            }
        }
    }
    cv::destroyWindow("Calibrated_foscam_image");

    cout << "****** Ensenso has captured and publishing cloud ******" << endl;

    get3DSensorPts(kinect_pts, pt_cloud_topic, img_topic,"foscam", src_cloud, image);

    ensenso_pub_cloud.request.get_cloud.data = false;
    bool flag = ensenso_pubcloud_serviceclient.call(ensenso_pub_cloud);

#endif

    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform("/ee_link", "/world", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/ee_link", "/world", ros::Time(0), transform);

    cout << "ee_link pts:\n";
    for(int i=0; i<robot_pts.size(); i++)
    {
        PointXYZ point = robot_pts[i];
        tf::Vector3 pt(point.x,point.y,point.z);
        tf::Vector3 tf_pt = transform*pt;

        PointXYZ tf_point;
        tf_point.x = tf_pt.getX();
        tf_point.y = tf_pt.getY();
        tf_point.z = tf_pt.getZ();
        cout << "{" << tf_point.x << ", " << tf_point.y << ", " << tf_point.z << "}" << endl;
        rbt.push_back(tf_point);

//        string frame_ee = "/ee_link";
//        double color_y[3] = {1.0,1.0,0.0};
//        double pt_vector[3] = {tf_point.x, tf_point.y, tf_point.z};
//        addMarker(pt_vector,color_y,sphere_marker,frame_ee, 2*i+1);
//        markerArray.markers.push_back(sphere_marker);
    }

    cout << "camera_link pts:\n";
    for(int i=0; i<kinect_pts.size(); i++)
    {
        PointXYZ point = kinect_pts[i];
        tf::Vector3 pt(point.x,point.y,point.z);

        tf::Vector3 tf_pt = pt;

        PointXYZ tf_point;
        tf_point.x = tf_pt.getX();
        tf_point.y = tf_pt.getY();
        tf_point.z = tf_pt.getZ();

        cout << "{" << tf_point.x << ", " << tf_point.y << ", " << tf_point.z << "}" << endl;
        knt.push_back(tf_point);

//        string frame_cl = "/camera_link";
//        double color_r[3] = {1.0,0.0,0.0};
//        double pt_vector[3] = {tf_point.x, tf_point.y, tf_point.z};
//        addMarker(pt_vector,color_r,sphere_marker,frame_cl, 2*i);
//        markerArray.markers.push_back(sphere_marker);
    }

//    pub_marker.publish(markerArray);

    Eigen::Matrix4f t;
    pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
    svd_estimator.estimateRigidTransformation(knt, rbt, t);
    std::cout << "sensor to rbt" << std::endl;
    std::cout << t << std::endl;

    svd_estimator.estimateRigidTransformation(rbt, knt, t);
    std::cout << "rbt to sensor" << std::endl;
    std::cout << t << std::endl;

    cout << "\n Make use of sensor to rbt Transform matrix in calibration_transform_setup in calibration package" << endl;

    return 0;
}


