#include <iostream>
#include "lines_rack_det/rackdetect.h"
#include "lines_rack_det/DetectShelf.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <signal.h>

// 0: Run rack detection in test mode
// 1: Run the service mode for rack detection
#define SERVICE_MODE 1
#define SHOW_CLOUD 1

// 0: Use live data from kinect for testing rack detection
// 1: Use a saved pt cloud of rack for testing
#define USE_SAVED_CLOUD 0 // used only if SERVICE_MODE is 0
string file = ros::package::getPath("lines_rack_det").append("/../../database/lines_rack/rack_ref_verror1.pcd");// saved cloud file name
//string file = "/home/ravi/ros_projects/apc_ws/src/database/lines_rack/rack_ref_verror1.pcd";

bool get_cloud = true;

#if(SERVICE_MODE)
RackDetect rack_detect;
//#if(SHOW_CLOUD)
//pcl::visualization::PCLVisualizer  service_pcl_visualizer;
//#endif
#endif

std::vector<pcl::PointXYZ> tf_bin_centroids(12), tf_bin_corners(48), tf_rack_corners(20);
bool gen_rack_data = false;

bool sendDataCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res);
bool detectRackCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res);
void kinectToWAMBase(std::vector<Eigen::Vector4d> &points, std::vector<pcl::PointXYZ> &tf_points);

bool rackDetect(RackDetect &rack, visualization::PCLVisualizer &pcl_visualizer);
void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, PointCloud<PointXYZRGBA>::Ptr &cloud_ptr, bool &copy);
bool rackCornerCentroidDetect(RackDetect &rack);

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;
    rack_detect.video.release();
    sleep(1);
    exit(signum);
}

int main (int argc, char** argv)
{
    ros::init(argc,argv,"rack_detect");
    ros::NodeHandle node;

    ros::ServiceServer service_detect_shelf = node.advertiseService("/lines_rack_detect", detectRackCallback);
    ros::ServiceServer service_transfer_info = node.advertiseService("/lines_rack_detect/send_data", sendDataCallback);

#if(SERVICE_MODE)
    ros::Subscriber sub_cloud = node.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
                                                                         boost::bind(kinectPCCallback, _1, boost::ref(rack_detect.src_cloud),
                                                                                     boost::ref(get_cloud)));
    while(ros::ok())
    {
        signal(SIGINT, signal_callback_handler);
        ros::spinOnce();
    }
#else
    RackDetect rack_detect;
    ros::Subscriber sub_cloud = node.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
                                                                         boost::bind(kinectPCCallback, _1, boost::ref(rack_detect.src_cloud),
                                                                                     boost::ref(get_cloud)));

    while(ros::ok()) // keep running until 'q' or Esc key is pressed
    {
        pcl::visualization::PCLVisualizer  pcl_visualizer("Rack corners centroids");
        pcl_visualizer.addCoordinateSystem(0.1, 0.0, 0.0, 0.0);
        rackDetect(rack_detect, pcl_visualizer);
        while(!pcl_visualizer.wasStopped() && ros::ok())
            pcl_visualizer.spinOnce();
    }
#endif

 return (0);
}

void kinectToWAMBase(std::vector<Eigen::Vector4d> &points, std::vector<pcl::PointXYZ> &tf_points)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), transform_kinect_j1);

//    cout << "Found transform" << endl;
//    cout << "size: "<< tf_points.size() << endl;
    for(int i=0; i<points.size(); i++)
    {
//        if(isnan(points[i](0)))
//            cout << "NaN at: "<< i << endl;
        tf::Vector3 pt(points[i](0), points[i](1), points[i](2));
        tf::Vector3 tf_pt = transform_kinect_j1*pt;
        pcl::PointXYZ tf_point(tf_pt.getX(), tf_pt.getY(), tf_pt.getZ());
//        tf_points.push_back(tf_point);
        tf_points[i] = tf_point;
//        cout << i << " " << endl;

    }


    return;
}

bool detectRackCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res)
{
    cout << "Service Reached: Lines rack detection" << endl;
//    RackDetect rack_detect;
//    pcl::visualization::PCLVisualizer  pcl_visualizer;

    double time = ros::Time::now().toSec();
    bool good_to_go = false;
    int it = 0;
    int max_iterations = 50;
    while(it < max_iterations && !good_to_go && ros::ok())
    {
        bool success = false;
        success = rackCornerCentroidDetect(rack_detect);

        it++;
        cout << "Iteration number: " << it << ", Horizontal: " << rack_detect.no_horiz_lines << ", Vertical: " << rack_detect.no_vert_lines << endl;

        if(success)
        {
            cout << "Found the rack corners" << endl;
            good_to_go = true;
            break;
        }
        else
            continue;
    }

    if(!good_to_go || it==max_iterations)
    {
        cout << "Could not get the rack corners and centroids" << endl;
        cout << "Exceeded iteration number of " << max_iterations << endl;
        return false;
    }
    else
    {
        cout << "Rack to bin corners and bin centroids are computed" << endl;
//        pcl::visualization::PCLVisualizer  service_pcl_visualizer_new;
#if(SHOW_CLOUD)
//        service_pcl_visualizer_new.removeAllPointClouds();
//        service_pcl_visualizer_new.removeAllShapes();
//        service_pcl_visualizer_new.addPointCloud(rack_detect.ref_cloud, "source cloud");
#endif
        float radii = .02;
        char id[10];
        string cor_id = "corner_";

        for(int i = 0; i<rack_detect.rack_corners.size() ;i++)
        {
            pcl::PointXYZ point;
            point.x = rack_detect.rack_corners[i](0);
            point.y = rack_detect.rack_corners[i](1);
            point.z = rack_detect.rack_corners[i](2);
            string str = cor_id;
            sprintf(id,"_%d",i);
            str.append(id);
#if(SHOW_CLOUD)
//            service_pcl_visualizer_new.addSphere<pcl::PointXYZ>(point, radii, 255.0, 0.0, 0.0, str);
#endif
        }

        for(int i = 0; i<rack_detect.bin_centroids.size() ;i++)
        {
            pcl::PointXYZ point;
            point.x = rack_detect.bin_centroids[i](0);
            point.y = rack_detect.bin_centroids[i](1);
            point.z = rack_detect.bin_centroids[i](2);
            string str = cor_id;
            sprintf(id,"_%d",i+20);
            str.append(id);
#if(SHOW_CLOUD)
//            service_pcl_visualizer_new.addSphere<pcl::PointXYZ>(point, radii, 0.0, 255.0, 0.0, str);
#endif
        }
        cout << endl << "Found rack corners and centroids at iteration number: " << it << endl;
        cout << "Found in: " << ros::Time::now().toSec()-time << " secs" << endl;



        /** Send out successful data **/
        gen_rack_data = true;
        cout << "Bin centroids size: " << rack_detect.bin_centroids.size() << " " << tf_bin_centroids.size() << endl;
        kinectToWAMBase(rack_detect.bin_centroids,tf_bin_centroids);
        cout << "Transformed centroids" << endl;
        cout << "rack corners size: " << rack_detect.rack_corners.size() << " " << tf_rack_corners.size() << endl;

        kinectToWAMBase(rack_detect.rack_corners,tf_rack_corners);
        cout << "Transformed rack corners" << endl;
        std::vector<Eigen::Vector4d> bin_corners(48);
        for(int i=0; i<rack_detect.bin_corners.size(); i++)
        {
            for(int j=0; j<rack_detect.bin_corners[i].size(); j++)
                bin_corners[i*4+j] = rack_detect.bin_corners[i][j];
        }
        cout << "Copied bin corners" << endl;
        kinectToWAMBase(bin_corners,tf_bin_corners);
        cout << "Transformed bin corners" << endl;

        res.centroids.data.clear();
        for(int i = 0; i < tf_bin_centroids.size(); i++)
        {
            cout << i << ": [" << tf_bin_centroids[i].x << " " << tf_bin_centroids[i].y << " " << tf_bin_centroids[i].z << "]" << endl;
            res.centroids.data.push_back(tf_bin_centroids[i].x);
            res.centroids.data.push_back(tf_bin_centroids[i].y);
            res.centroids.data.push_back(tf_bin_centroids[i].z);
        }
        cout << "Transformed bin corners: \n" ;
        res.bin_corners.data.clear();
        for(int i = 0; i < tf_bin_corners.size(); i++)
        {
            cout << i << " ";
            res.bin_corners.data.push_back(tf_bin_corners[i].x);
            res.bin_corners.data.push_back(tf_bin_corners[i].y);
            res.bin_corners.data.push_back(tf_bin_corners[i].z);
        }
        cout << "came out" << endl;
#if(SHOW_CLOUD)
//        while(!service_pcl_visualizer_new.wasStopped() && ros::ok())
//            service_pcl_visualizer_new.spinOnce(2000);// this command has to be there for displaying the cloud
#endif
//        service_pcl_visualizer_new.close();
        cout << "Returning from lines rack detection service" << endl;
        return true;
    }

}

bool sendDataCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res)
{
    cout << "Reached shelf data transfer service" << endl;
    if(gen_rack_data)
    {
        srand(0);
        for(int i = 0; i < tf_bin_centroids.size(); i++)
        {
            res.centroids.data.push_back(tf_bin_centroids[i].x);
            res.centroids.data.push_back(tf_bin_centroids[i].y);
            res.centroids.data.push_back(tf_bin_centroids[i].z);
//            cout << i << "[" << tf_bin_centroids[i].x << " " << tf_bin_centroids[i].y << " " << tf_bin_centroids[i].z << "]" << endl;
        }
//        cout << endl;
        for(int i = 0; i < tf_bin_corners.size(); i++)
        {
            res.bin_corners.data.push_back(tf_bin_corners[i].x);
            res.bin_corners.data.push_back(tf_bin_corners[i].y);
            res.bin_corners.data.push_back(tf_bin_corners[i].z);
//            cout << i << "[" << tf_bin_corners[i].x << " " << tf_bin_corners[i].y << " " << tf_bin_corners[i].z << "]" << endl;
        }
//        cout << "Transformed shelf data wrt WAM base" << endl;
        return true;
    }
    else
    {
        cout << "Shelf data not yet generated" << endl;
        return false;
    }
}

bool rackDetect(RackDetect &rack, pcl::visualization::PCLVisualizer  &pcl_visualizer)
{
    static int count = 0;
    double time = ros::Time::now().toSec();
    bool good_to_go = false;
    int it = 0;
    int max_iterations = 100;
    while(it < max_iterations && !good_to_go && ros::ok())
    {
        cout << "<------start-------->" << endl;
        bool success = false;
        success = rackCornerCentroidDetect(rack);

        it++;

        cout << "Iteration number: " << it << ", Horizontal: " << rack.no_horiz_lines << ", Vertical: " << rack.no_vert_lines << endl;


        if(success)
        {
            cout << "Found the rack corners" << endl;
            good_to_go = true;
            break;
        }
        else
        {
            cout << "<-------end------->" << endl;
            continue;
        }

//        // check whether no of horizontal lines are 3 and distance between vertical lines is greater than 0.75m- MIN WIDTH of rack
//        if(rack.no_horiz_lines == H_LINES-2)
//        {
//            good_to_go = true;
//            for(int j=0; j<rack.rack_corners.size(); j++)
//            {
//                Eigen::Vector4d pt = rack.rack_corners[j];
//                if(isnan(pt(0)))
//                {
//                    good_to_go = false;
//                    cout << "Got NaN, going to find rack again" << endl;
//                    cout << "Iteration number " << it << endl;
//                    break;
//                }
//            }

//            if(good_to_go)
//            {
//                // test for distance between vertical lines is greater than 0.8m
//                double l_cor = 0, r_cor = 0;
//                for(int j=0; j<H_LINES; j++)
//                {
//                    l_cor += rack.rack_corners[ROWS*j](0);
//                    r_cor += rack.rack_corners[ROWS*j+COLUMNS](0);
//                }
//                l_cor /= H_LINES;
//                r_cor /= H_LINES;
//                if(fabs(r_cor-l_cor) < RACK_WIDTH_MIN)
//                {
//                    cout << endl << "Found vertical lines less than " << RACK_WIDTH_MIN << "m" << endl << endl;
//                    good_to_go = false;
//                }
//            }
//        }
    }

    count++;
    cout << "Count num: " << count << endl;

    if(!good_to_go || it==max_iterations)
    {
        cout << "Could not get the rack corners and centroids" << endl;
        cout << "Exceeded iteration number of " << max_iterations << endl;
        return false;
    }
    else
    {
        cout << "Rack to bin corners and bin centroids are computed" << endl;

//        pcl::visualization::PCLVisualizer  pcl_visualizer;
//        pcl_visualizer.addCoordinateSystem(0.1, 0.0, 0.0, 0.0);

        pcl_visualizer.removeAllPointClouds();
        pcl_visualizer.removeAllShapes();
//        if(count > 1)
//            pcl_visualizer.updatePointCloud(rack.ref_cloud,"source cloud");
//        else
            pcl_visualizer.addPointCloud(rack.ref_cloud, "source cloud");

        float radii = .02;
        char id[10];
        string cor_id = "corner_";

        for(int i = 0; i<rack.rack_corners.size() ;i++)
        {
            pcl::PointXYZ point;
            point.x = rack.rack_corners[i](0);
            point.y = rack.rack_corners[i](1);
            point.z = rack.rack_corners[i](2);
            string str = cor_id;
            sprintf(id,"_%d",i);
            str.append(id);
            pcl_visualizer.addSphere<pcl::PointXYZ>(point, radii, 255.0, 0.0, 0.0, str);
        }

        for(int i = 0; i<rack.bin_centroids.size() ;i++)
        {
            pcl::PointXYZ point;
            point.x = rack.bin_centroids[i](0);
            point.y = rack.bin_centroids[i](1);
            point.z = rack.bin_centroids[i](2);
            string str = cor_id;
            sprintf(id,"_%d",i+20);
            str.append(id);
            pcl_visualizer.addSphere<pcl::PointXYZ>(point, radii, 0.0, 255.0, 0.0, str);
        }
        cout << endl << "Found rack corners and centroids at iteration number: " << it << endl;
        cout << "Found in: " << ros::Time::now().toSec()-time << " secs" << endl;


        pcl_visualizer.spinOnce();// this command has to be there for displaying the cloud
        return true;
    }

}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, PointCloud<PointXYZRGBA>::Ptr &cloud_ptr, bool &copy)
{

//    cout << "Reached kinect call back" << endl;
//    cout << copy << endl;
    if(copy)
    {
        cout << "copying inside call back" << endl;
        pcl::fromROSMsg(*input, *cloud_ptr);
        if(!cloud_ptr->empty())
        {
            copy = false;
            cout << "Copied cloud in kinect call back " << endl;
        }
    }
    return;
}

bool rackCornerCentroidDetect(RackDetect &rack)
{
#if(USE_SAVED_CLOUD)
    // Load the point cloud between which transformation has to be found
    int out = io::loadPCDFile(file,*rack.src_cloud);
    cout << "Loaded point cloud " << file << endl;
    if(rack.src_cloud->empty() || out < 0) // out>0 if file is opened properly
    {
        if(out<0)
            cerr << file << " could not be opened ...\n" << endl;
        else
            cerr << file << " point cloud is empty ..." << endl;
        exit(0);
    }
#else
    get_cloud = true;
    while(get_cloud && ros::ok())
    {
        get_cloud = true;
        ros::spinOnce();
//        if(rack.get_cloud && ros::ok())
//            cout << "Cloud not found" << endl;
//        cout << "Press enter" ;
//        getchar();
    }

//    cout << "Cloud found" << endl;

#endif

    rack.createRackImageCloud(rack.src_cloud);

    if(rack.src_cloud->empty())// check if no point cloud is obtained
    {
#if(DEBUG_MODE)
        cout << "Point cloud is empty" << endl;
#endif
        return false;
    }
    // write the cropped reference image
//    rack.video.write(rack.ref_image);
    static int img_c = 0;
    char s[200];
    sprintf(s,"/home/ilab/tcs/apc_ws/src/apc_project/lines_rack_det/data/ref_img_%d.jpg",img_c);
    cv::imwrite(s, rack.ref_image);
    img_c++;

    string saveFile = ros::package::getPath("lines_rack_det").append("/../../database/lines_rack/rack_ref_new.pcd");
    pcl::io::savePCDFileASCII(saveFile,*rack.ref_cloud);
    cout << "Saved:\n\t" << saveFile << endl;
#if(DEBUG_MODE)
    cout << "Created rack image" << endl;
    char c = (char) waitKey(1);

    imshow("crop_image",rack.ref_image);

    if(c=='q' || c==27)
    {
        cout << "ROS shutdown" << endl;
        ros::shutdown();
    }

#endif

    vector<Vec4i> lines;
    vector<Vec4i> hfl, vfl;
    rack.getHoughLines(rack.ref_image, lines);

    if(lines.empty())// check if no lines are obtained
    {
#if(DEBUG_MODE)
        cout << "No lines are found" << endl;
#endif
        return false;
    }

#if(DEBUG_MODE)
    cout << "Total no. of lines found: " << lines.size() << endl;
#endif
//  Filter the lines into 3 horizontal lines and 2 vertical lines
    rack.getVerHorLines(lines,vfl,hfl);

#if(DEBUG_MODE)
    cout << "Vertical lines: " << vfl.size() << endl;
    cout << "horizontal lines: " << hfl.size() << endl;
#endif

    rack.no_horiz_lines = hfl.size();
    rack.no_vert_lines = vfl.size();

    if(hfl.size() != 3 || vfl.size() !=2)// check whether 3 horizontal lines are 2 vertical lines are found or not
    {
#if(DEBUG_MODE)
        if(hfl.size() != 3)
            cout << "No. of horizontal lines found is " << hfl.size() << endl;
        else
            cout << "No. of vertical lines found is " << vfl.size() << endl;
#endif
        return false;
    }

    // check whether distance between vertical lines is greater that 75cm or not
    vector<Vec2i> v_end_pts(vfl.size());
    vector<pcl::PointXYZ> v_end3d(vfl.size());
    for(int i=0; i<vfl.size(); i++)
    {
        int y1 = vfl[i](0), y2 = vfl[i](2);
        int x1 = vfl[i](1), x2 = vfl[i](3);

        v_end_pts[i](0) = (x1+x2)/2;
        v_end_pts[i](1) = (y1+y2)/2;
        bool success = rack.point2Dto3D(v_end_pts[i], v_end3d[i]);
    }
    if(fabs(v_end3d[0].x-v_end3d[1].x) < 0.8)// distance between vertical poles of rack is 0.85m
        return false;
    else
        cout << "Vertical poles are at distance: " << fabs(v_end3d[0].x-v_end3d[1].x) << endl;

    // 6 intersection points are found, first 3 are on left vertical and next 3 are on right vertical
    vector<Vec2i> inter_points;
    rack.getIntersections(vfl,hfl,inter_points);

#if(DEBUG_MODE)
    cout << "Got intersection points" << endl;
#endif
    Mat img;
    rack.ref_image.copyTo(img);

    vector<Vec3d> cluster_pts;
    for(int i=0; i<inter_points.size(); i++)
    {
        PointXYZ pt;
        bool success = rack.point2Dto3D(inter_points[i], pt);
        if(!success)
        {
            cout << "NaN at intersection point: " << i << endl;
            return false;
        }
        Vec3d pt3d(pt.x,pt.y,pt.z);
        cluster_pts.push_back(pt3d);

        circle(img, Point(inter_points[i][1],inter_points[i][0]),12, Scalar(0,0,255),CV_FILLED, 8,0);
    }
//    circle(img, Point((inter_points[0][1]+inter_points[1][1])*2/3,inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    circle(img, Point(inter_points[i][1],inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    circle(img, Point(inter_points[i][1],inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    circle(img, Point(inter_points[i][1],inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    circle(img, Point(inter_points[i][1],inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    circle(img, Point(inter_points[i][1],inter_points[i][0]),5, Scalar(155,255,155),CV_FILLED, 8,0);
//    rack.video.write(img);

    static int img2_c = 0;
    sprintf(s,"/home/ilab/tcs/apc_ws/src/apc_project/lines_rack_det/data/final_%d.jpg",img2_c);
    img2_c++;
    cv::imwrite(s, img);

    imshow("Corners points", img);
    waitKey(1);

    //    rack.clusterPoints(inter_3d, cluster_pts);
#if(DEBUG_MODE)
    cout << "No of intersecting points: " << cluster_pts.size() << endl;
#endif

    rack.rearrangeClusters(cluster_pts);
    if(cluster_pts.size() == 6)
    {
        rack.formRackCorners(cluster_pts);
        rack.racktoBinCorners(rack.rack_corners, rack.bin_corners, rack.bin_centroids);
        return true;
    }
    else
        return false;
}

