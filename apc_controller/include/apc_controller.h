#ifndef APC_CONTROLLER_H
#define APC_CONTROLLER_H

#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>

#include <wam_ikfast_7dof_service/PoseJoint7dof.h>
#include <lines_rack_det/DetectShelf.h>
//#include <apc_controller/kinectRobotTf.h>

#include <json_maker/write_pick_status.h>
#include <json_maker/write_stow_data.h>
#include <json_maker/get_bin_object.h>

#include <json_maker/getBinContents.h>

#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>

#define TRAJECTORY_SIMULATION  0
#define ROBOT_ANGLE_SIMULATION  1
#define ROBOT      1
#define NO_OF_BINS  12
#define NO_OF_JOINTS    16

#define VACCUM_SUCTION  1//  Vaccum suction flag

#define XWAM_MOTION 1 // Moving xwam forward backward and up down flag

#define SHOW_SIMULATION_MARKER  1

#define OBJECT_BIN          1 // To enable object bin

#define STOW_TASK           1 // To pick object from tote

#define WEBCAM_TOTE_VIDEO_RECORD 0


#define TRAJECTORY_RPT 1

#if(STOW_TASK)

#define DEFAULT_TOTE_OBJ_POSE 0
#endif

#if(SHOW_SIMULATION_MARKER)
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "wam_ikfast_7dof_service/wam_ik_f7.h"
#endif

#if(ROBOT)
#include<apc_controller/JointMove.h>
//#include<apc_controller/JointMoveSam.h>
#endif


#if(XWAM_MOTION)
#include <xwamotion_laser/FwdBck.h>
#include <xwamotion_laser/UpDown.h>
#endif

#if(TRAJECTORY_RPT)
#include <trajectory_rpt/TrajRpt.h>
#include <trajectory_rpt/TrajStow.h>
#include <trajectory_rpt/ContinuousTrajGen.h>
#endif

// OBJECT DETECTION APPROACHES
#define RCNN 1

#if(RCNN)
#include <apc_controller/objectDetect.h>
#include <apc_controller/stowObjDetect.h>
#include <json_maker/stowToteContents.h>
#endif

#define RACK_REGISTRATION 1
#define SEND_END_CORNERS 1
#if(RACK_REGISTRATION)
#include <apc_controller/rack_registration.h>
#endif


#define PICKING_TASK    1

using namespace std;

std::string model_names[] = {

    "dove_beauty_bar",          "rawlings_baseball",                    "clorox_utility_brush",         "dr_browns_bottle_brush",   "dasani_water_bottle",
    "easter_turtle_sippy_cup",  "cherokee_easy_tee_shirt",              "folgers_classic_roast_coffee", "crayola_24_ct",            "peva_shower_curtain_liner",
    "barkely_hide_bones",       "kyjen_squeakin_eggs_plush_puppies",    "expo_dry_erase_board_eraser",  "scotch_duct_tape",         "jane_eyre_dvd",
    "scotch_bubble_mailer",     "woods_extension_cord",                 "womens_knit_gloves",           "cool_shot_glue_sticks",    "elmers_washable_no_run_school_glue",
    "staples_index_cards",      "laugh_out_loud_joke_book",             "i_am_a_bunny_book",            "kleenex_tissue_box",       "soft_white_lightbulb",
    "kleenex_paper_towels",     "rolodex_jumbo_pencil_cup",             "ticonderoga_12_pencils",       "platinum_pets_dog_bowl",   "hanes_tube_socks",
    "creativity_chenille_stems","fiskars_scissors_red",                 "cloud_b_plush_bear",           "safety_first_outlet_plugs","fitness_gear_3lb_dumbbell",
    "oral_b_toothbrush_green",  "up_glucose_bottle",                    "command_hooks",                "oral_b_toothbrush_red", "none"
};

void publishJointStateInSimulation(vector<double> &jointVector, ros::Publisher &simJointPublisher)
{


    sensor_msgs::JointState simJointState;
    simJointState.name.resize(int(NO_OF_JOINTS));
    simJointState.position.resize(int(NO_OF_JOINTS));

    ros::Rate loop_rate(10);

    simJointState.name[0] ="joint_front_right_steer";
    simJointState.name[1] ="joint_front_right_wheel";
    simJointState.name[2] ="joint_front_left_steer";
    simJointState.name[3] ="joint_front_left_wheel";
    simJointState.name[4] ="joint_back_left_steer";
    simJointState.name[5] ="joint_back_left_wheel";
    simJointState.name[6] ="joint_back_right_steer";
    simJointState.name[7] ="joint_back_right_wheel";
    simJointState.name[8] ="scissor_prismatic_joint";
    simJointState.name[9] ="j1_joint";
    simJointState.name[10] ="j2_joint";
    simJointState.name[11] ="j3_joint";
    simJointState.name[12] ="j4_joint";
    simJointState.name[13] ="j5_joint";
    simJointState.name[14] ="j6_joint";
    simJointState.name[15] ="j7_joint";

    for(int i=0;i<8;i++)
        simJointState.position[i] = 0.0;

    simJointState.position[8] = 0.5;

    for(int i=9;i<16;i++)
        simJointState.position[i] = jointVector[i-jointVector.size()-2];

    for(int i = 0; i < 10; i++)
    {
        simJointPublisher.publish(simJointState);
        loop_rate.sleep();

    }

    return ;
}

#if(SHOW_SIMULATION_MARKER)
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

void getWAMBaseToKinect(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
//    tf::StampedTransform transform_kinect_j1

    transform_listener.waitForTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), transform);
    return;
}

void wamBaseToKinect(vector<double> &point, vector<double> &tf_point)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), transform_kinect_j1);

    {
        tf::Vector3 pt(point[0], point[1], point[2]);
        tf::Vector3 tf_pt = transform_kinect_j1*pt;
        tf_point[0] = tf_pt.getX();
        tf_point[1] = tf_pt.getY();
        tf_point[2] = tf_pt.getZ();
    }

    return;
}

void kinectToWAMBase(geometry_msgs::Point &point, geometry_msgs::Point &tf_point)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), transform_kinect_j1);

    tf::Vector3 pt(point.x,point.y,point.z);
    tf::Vector3 tf_pt = transform_kinect_j1*pt;

    tf_point.x = tf_pt.getX();
    tf_point.y = tf_pt.getY();
    tf_point.z = tf_pt.getZ();

    return;
}

// Function to transform a vector from kinect frame to WAM base frame
void kinectToWAMVector(geometry_msgs::Point &point, geometry_msgs::Point &tf_point)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/wam_base_link", "/camera_depth_optical_frame", ros::Time(0), transform_kinect_j1);

    tf::Vector3 pt(point.x,point.y,point.z);
    tf::Vector3 tf_pt = transform_kinect_j1*pt;

    tf::Vector3 tr_origin = transform_kinect_j1.getOrigin();

    tf_point.x = tf_pt.getX()-tr_origin.getX();
    tf_point.y = tf_pt.getY()-tr_origin.getY();
    tf_point.z = tf_pt.getZ()-tr_origin.getZ();

    return;
}

void wamBasetoDepthFrameTr()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform_kinect_j1;

    transform_listener.waitForTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/camera_depth_optical_frame", "/wam_base_link", ros::Time(0), transform_kinect_j1);

    cout << "Wam base link to camera_depth_optical frame transform: \n";
    tf::Matrix3x3 rotation = transform_kinect_j1.getBasis();

    cout << rotation.getRow(0).getX() << " " << rotation.getRow(0).getY() << " " << rotation.getRow(0).getZ() << endl;
    cout << rotation.getRow(1).getX() << " " << rotation.getRow(1).getY() << " " << rotation.getRow(1).getZ() << endl;
    cout << rotation.getRow(2).getX() << " " << rotation.getRow(2).getY() << " " << rotation.getRow(2).getZ() << endl;
    cout << transform_kinect_j1.getOrigin().getX() << " " << transform_kinect_j1.getOrigin().getY() << " " << transform_kinect_j1.getOrigin().getZ() << endl;

    transform_listener.waitForTransform("/fixed_kinect", "/wam_base_link", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/fixed_kinect", "/wam_base_link", ros::Time(0), transform_kinect_j1);

    cout << "Wam base link to fixedkinect frame transform: \n";
    rotation = transform_kinect_j1.getBasis();

    cout << rotation.getRow(0).getX() << " " << rotation.getRow(0).getY() << " " << rotation.getRow(0).getZ() << endl;
    cout << rotation.getRow(1).getX() << " " << rotation.getRow(1).getY() << " " << rotation.getRow(1).getZ() << endl;
    cout << rotation.getRow(2).getX() << " " << rotation.getRow(2).getY() << " " << rotation.getRow(2).getZ() << endl;
    cout << transform_kinect_j1.getOrigin().getX() << " " << transform_kinect_j1.getOrigin().getY() << " " << transform_kinect_j1.getOrigin().getZ() << endl;

    transform_listener.waitForTransform("/wam_link4", "/wamj4_kinect_link", ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/wam_link4", "/wamj4_kinect_link", ros::Time(0), transform_kinect_j1);

    cout << "Wam link4 to j4tokinectlink frame transform: \n";
    rotation = transform_kinect_j1.getBasis();

    cout << rotation.getRow(0).getX() << " " << rotation.getRow(0).getY() << " " << rotation.getRow(0).getZ() << endl;
    cout << rotation.getRow(1).getX() << " " << rotation.getRow(1).getY() << " " << rotation.getRow(1).getZ() << endl;
    cout << rotation.getRow(2).getX() << " " << rotation.getRow(2).getY() << " " << rotation.getRow(2).getZ() << endl;
    cout << transform_kinect_j1.getOrigin().getX() << " " << transform_kinect_j1.getOrigin().getY() << " " << transform_kinect_j1.getOrigin().getZ() << endl;

    return;
}

void sphereMarker(vector<tf::Vector3> &points, ros::Publisher &marker_pub)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker sphere_marker;

    double color[3] = {1.0, 0.0, 0.0};

    double centroid[3];

    cout << "ref color: red" << endl;
    for(int j=0; j<points.size(); j++)
    {
        tf::Vector3 v = points[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = "/camera_depth_optical_frame";

        addMarker(centroid,color,sphere_marker,frame, j);
        markerArray.markers.push_back(sphere_marker);
    }

//    while(marker_pub.getNumSubscribers() < 1)
//    {
//        ros::Duration(1).sleep();
//        cout << "Put subscriber for topic: \bin_corner"  << endl;
//    }

    marker_pub.publish(markerArray);

    return;
}

void showMarkerInSimulation(vector<double> kinectCentroid,vector<double> wamCentroid,vector<double> fkCentroid, ros::Publisher &marker_pub)
{
    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker kinect_centroid_marker;

    double color[3] = {1.0, 0.0, 0.0};

    double centroid[3];

    ros::Rate loop_rate(10);

    for(int i=0;i<3;i++)
        centroid[i]=kinectCentroid[i];
    string frame = "/camera_depth_optical_frame";

    cout << "Kinect ref color: red" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 0);
    markerArray.markers.push_back(kinect_centroid_marker);

    for(int i=0;i<3;i++)
        centroid[i]=wamCentroid[i];

    color[0] = 0;
    color[1] = 1;
    color[2] = 0;
    frame = "/wam_base_link";

    cout << "wam ref color: green" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 1);
    markerArray.markers.push_back(kinect_centroid_marker);

//    for(int i=0;i<3;i++)
//        centroid[i]=fkCentroid[i];
    centroid[0]=fkCentroid[0]-0.22;
    centroid[1]=fkCentroid[1]-0.14;
    centroid[2]=fkCentroid[2]-0.346;

    color[0] = 0;
    color[1] = 0;
    color[2] = 1;

    cout << "FK ref color: blue" << endl;
    addMarker(centroid,color,kinect_centroid_marker,frame, 2);
    markerArray.markers.push_back(kinect_centroid_marker);

    while(marker_pub.getNumSubscribers() < 1)
    {
        ros::Duration(1).sleep();
        cout << "Put subscriber for kinect and WAM base marker" << endl;
    }

    marker_pub.publish(markerArray);

    return;
}

#endif

void wamJointStateCallback(const sensor_msgs::JointState::ConstPtr &jt_state_msg, vector<double> &jt_state)
{
    for(int i = 0; i < jt_state_msg->position.size(); i++)
        jt_state[i] = jt_state_msg->position[i];

    return;
}

bool checkJointsReached(vector<double> &desired_joints_state, vector<double> &current_joints_state)
{
    bool goal_achieved = false;
    double current_jts[7];
    double time = ros::Time::now().toSec();
    double duration;

    int start_jt = 1;
    while(!goal_achieved)
    {
        ros::spinOnce();
        for(int i = 0; i < current_joints_state.size(); i++)
            current_jts[i] = current_joints_state[i];

        double avg_pos_diff = 0;
        for(int i = start_jt; i < desired_joints_state.size(); i++)
            avg_pos_diff += fabs((current_jts[i] - desired_joints_state[i]));
        avg_pos_diff /= desired_joints_state.size();

        // wait until avg joint position diff is approx. 5.7 degrees
        if(avg_pos_diff < 0.025)
            goal_achieved = true;
        else
        {
            goal_achieved = false;

        }

        duration = ros::Time::now().toSec() - time;
        if(duration > 5) // if in loop for more than 60 secs then break out
        {
            ROS_INFO("Time limit exceeded to reach desired position");
            goal_achieved = true;
            break;
        }
//        cout << "Desired: [";
//        for(int i=0; i<7 ; i++)
//            cout << desired_joints_state[i] << " ";
//        cout << "]\n";
//        cout << "Current: [";
//        for(int i=0; i<7 ; i++)
//            cout << current_joints_state[i] << " ";
//        cout << "]";
    }
    if(goal_achieved)
    {
//        cout << "WAM desired position achieved in: " << duration << "seconds" << endl;
        return true;
    }
    else
        return false;
}


vector< vector<double> > centroidVector = vector< vector<double> >(int(NO_OF_BINS),vector<double>(3,0));
vector<tf::Vector3> bin_corners(4*(int (NO_OF_BINS)));
bool got_shelf_data = false;
bool shelfDataCallback(lines_rack_det::DetectShelf::Request &req, lines_rack_det::DetectShelf::Response &res)
{
    if(got_shelf_data)
    {
        cout << "/***** At shelf data transfer service *****/";
        for(int i = 0; i < centroidVector.size(); i++)
        {
            vector<double> v = centroidVector[i];
            for(int j = 0; j < v.size(); j++)
                res.centroids.data.push_back(v[j]);
        }
        for(int i = 0; i < bin_corners.size(); i++)
        {
            tf::Vector3 v = bin_corners[i];
            res.bin_corners.data.push_back(v.getX());
            res.bin_corners.data.push_back(v.getY());
            res.bin_corners.data.push_back(v.getZ());
        }
        cout << "Sent the shelf centroid and corner data" << endl;
        return true;
    }
    else
    {
        cout << "Shelf centroid and corner data not obtained" << endl;
        return false;
    }
}

// Function to modify centroids and corners in case the robot moves in x,y,z direction
// x,y,z all should be provided
// values should be positive in case moved along the positive direction of the axis
// values should be negative in case moved along the negative direction of the axis
void translateCentroidCorners(double x, double y, double z)
{
    for(int i=0; i<centroidVector.size(); i++)
    {
//        cout << "Centroid " << i << ": " << centroidVector[i][0] << "\t" << centroidVector[i][1] << "\t" << centroidVector[i][2] << "\t" << endl;
        centroidVector[i][0] -= x;
        centroidVector[i][1] -= y;
        centroidVector[i][2] -= z;
//        cout << "M Centroid " << i << ": " << centroidVector[i][0] << "\t" << centroidVector[i][1] << "\t" << centroidVector[i][2] << "\t" << endl;
    }
    tf::Vector3 v(x,y,z);
    for(int i=0; i<bin_corners.size(); i++)
    {
        bin_corners[i] -= v;
    }
//    if(x < 0)
//        cout << "Centroid are shifted right: " << x;
//    else if(x > 0)
//        cout << "Centroid are shifted left: " << x;
//    if(y < 0)
//        cout << "Centroid are shifted forward: " << y;
//    else if(y > 0)
//        cout << "Centroid are shifted backward: " << y;
//    if(z < 0)
//        cout << "Centroid are shifted up: " << z;
//    else if(z > 0)
//        cout << "Centroid are shifted down: " << z;

//    char c;
//    cout << "\nPress any character ..." << endl;
//    cin >> c;
//    getchar();
    return;
}

void kinectPCCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr, bool &copy)
{

//    cout << "Reached kinect call back" << endl;
//    cout << copy << endl;
    if(copy)
    {
//        cout << "copying inside call back" << endl;
        pcl::fromROSMsg(*input, *cloud_ptr);
        if(!cloud_ptr->empty())
        {
            copy = false;
//            cout << "Copied cloud in kinect call back " << endl;
        }

//        *output = *input;
//        if(!output->data.empty())
//        {
//            copy = false;
//            cout << "Copied cloud in kinect call back " << endl;
//        }
    }
    return;
}

void irSensorCallback(const std_msgs::Int16::ConstPtr &ir_msg, int &ir_data, bool &flag_available)
{
    ir_data = ir_msg->data;
    flag_available = true;
    return;
}

void metalSensorCallback(const std_msgs::Int16::ConstPtr &metal_msg, int &metal_data, bool &flag_available)
{
    metal_data = metal_msg->data;
    flag_available = true;
    return;
}

// Function to call /wam/joint_move  service
bool jtMoveServiceCall(vector<double> &jts, ros::Publisher &simJoints, ros::ServiceClient &service, vector<double> &current_jts, string success_msg)
{
    // copy the jt angles for tote center view position
    apc_controller::JointMove jt_move;
    for(int i=0; i<jts.size(); i++)
    {
        jt_move.request.joints.push_back(jts[i]);
    }

//    apc_controller::JointMove jt_move;
//    for(int i=0; i<jts.size(); i++)
//    {
//        jt_move.request.joints.push_back(jts[i]);
//        jt_move.request.joint_vel = 1.0;
//    }

    // publish command to reach tote center view position
    publishJointStateInSimulation(jts,simJoints);

    if(service.call(jt_move))
    {
        vector<double> wam_desired_jts(jts.size(),0.0);

        for(int i = 0; i < wam_desired_jts.size(); i++)
            wam_desired_jts[i] = jt_move.request.joints[i];
        if(checkJointsReached(wam_desired_jts,current_jts))
        {
//            cout << success_msg.c_str() << endl;
            return true;
        }
        else
        {
            cout << "Failed to reach desired wam position" << endl;
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return false;
    }
}

#endif // APC_CONTROLLER_H
