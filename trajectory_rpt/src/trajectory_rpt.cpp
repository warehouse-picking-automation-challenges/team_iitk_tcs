#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include "trajectory_rpt/TrajRpt.h"
#include "trajectory_rpt/trajectory_rpt.h"

using namespace std;
#define NORMAL_DEF 0
#define DEFAULT_MIDDLE_FRONT_POSITION 0
#define DEFAULT_LEFT_POSITION 0
#define DEFAULT_RIGHT_POSITION 0

enum pick_direction {middle_bottom, middle_front, left_side, right_side};
enum obj_position {psn_middle, psn_left, psn_right};

// could take into consideration of gap to be maintained
// function to refine the eef for picking the object
// robot rotated by 90deg in z-direction
void refineeef2(geometry_msgs::Point &centroid, geometry_msgs::Point &normal_vect, vector<geometry_msgs::Point> &corners, geometry_msgs::Point &refined_centroid,
                pick_direction &pdir)
{
    tf::Vector3 eef_psn(centroid.x, centroid.y, centroid.z);
    tf::Vector3 eef_normal(normal_vect.x, normal_vect.y, normal_vect.z);
//    tf::Vector3 refined_psn;
    // lc: left corners avg, rc: right corners avg, tc, top corners avg, bc, bottom corners avg
    geometry_msgs::Point lc, rc, tc, bc;
    lc.x = (corners[2].x + corners[0].x)/2;
    lc.y = (corners[2].y + corners[0].y)/2;
    lc.z = (corners[2].z + corners[0].z)/2;

    rc.x = (corners[1].x + corners[3].x)/2;
    rc.y = (corners[1].y + corners[3].y)/2;
    rc.z = (corners[1].z + corners[3].z)/2;

    tc.x = (corners[0].x + corners[1].x)/2;
    tc.y = (corners[0].y + corners[1].y)/2;
    tc.z = (corners[0].z + corners[1].z)/2;

    bc.x = (corners[2].x + corners[3].x)/2;
    bc.y = (corners[2].y + corners[3].y)/2;
    bc.z = (corners[2].z + corners[3].z)/2;

    obj_position obj_psn;
    // find eef_psn is middle, left or right side
    if(fabs(lc.x - eef_psn.x()) < 0.06)
        obj_psn = psn_left;
    else if(fabs(rc.x - eef_psn.x()) < 0.06)
        obj_psn = psn_right;
    else
        obj_psn = psn_middle;

    // find the picking direction
    if(obj_psn == psn_middle)
    {
        if(fabs(eef_psn.z() - tc.z) > 0.12)
            pdir = middle_bottom;
        else
            pdir = middle_front;
    }
    else if(obj_psn == psn_left)
        pdir = left_side;
    else
        pdir = right_side;

    // refine the eef position so that the arm won't collide with the bin
    refined_centroid.x = eef_psn.x();
    refined_centroid.y = eef_psn.y();
    refined_centroid.z = eef_psn.z();
    if(pdir == middle_bottom)
    {
        tf::Vector3 robot_base_ny(0,-1,0);
        if(eef_normal.dot(robot_base_ny) > 0.5) // if normal is along -ve y-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02;
    }
    else if(pdir == middle_front)
    {
        if(fabs(eef_psn.z() - bc.z) < 0.06)// if obj needs to be picked from front then the centroid should be atleast 6cm above bin base
            refined_centroid.z = bc.z + 0.06;
    }
    else if(pdir == left_side)
    {
        tf::Vector3 robot_base_x(1,0,0);
        if(eef_normal.dot(robot_base_x) < 0.5)// if normal is not along x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02;
    }
    else if(pdir == right_side)
    {
        tf::Vector3 robot_base_nx(-1,0,0);
        if(eef_normal.dot(robot_base_nx) < 0.5)// if normal is not along -ve x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02;
    }
    return;
}

bool getTrajCallback(trajectory_rpt::TrajRpt::Request &req, trajectory_rpt::TrajRpt::Response &res)
{
    std::cout << "Reached service for trajectory planning rpt" << std::endl;

    double x_offset = 0.03, y_offset = -0.01, z_offset = 0.03;

    geometry_msgs::Point bin_centroid;
    bin_centroid.x = req.bin_centroid.data[0];
    bin_centroid.y = req.bin_centroid.data[1];
    bin_centroid.z = req.bin_centroid.data[2];

    vector<geometry_msgs::Point> bin_corners(4);
    int idx=0;
    for(int i=0; i<4; i++)
    {
        bin_corners[i].x = req.bin_corners.data[idx++];
        bin_corners[i].y = req.bin_corners.data[idx++];
        bin_corners[i].z = req.bin_corners.data[idx++];
    }

    int bin_num = req.bin_num.data;
    geometry_msgs::Point obj_centroid;
    obj_centroid.x = req.obj_centroid.x + x_offset;
    obj_centroid.y = req.obj_centroid.y + y_offset;
    obj_centroid.z = req.obj_centroid.z + z_offset;

    geometry_msgs::Point obj_normal;
    obj_normal.x = req.obj_normal.x;
    obj_normal.y = req.obj_normal.y;
    obj_normal.z = req.obj_normal.z;

    cout << "With x_offset " << x_offset << endl;
    pick_direction pk_dir;
#if(DEFAULT_MIDDLE_FRONT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += x_offset;
    obj_centroid.y += y_offset+0.05;
    obj_centroid.z += z_offset;
    pk_dir = middle_front;
#elif(DEFAULT_LEFT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += x_offset - 0.05;
    obj_centroid.y += y_offset+0.05;
    obj_centroid.z += z_offset;
    pk_dir = left_side;
#elif(DEFAULT_RIGHT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += x_offset + 0.05;
    obj_centroid.y += y_offset+0.05;
    obj_centroid.z += z_offset;
    pk_dir = right_side;
#else
    // Obtain object placing position and picking direction
//    refineeef2(obj_centroid, obj_normal, bin_corners, obj_centroid, pk_dir);
    pk_dir = middle_front;
#endif

    cout << "Bin centroid: " << bin_centroid.x << " " << bin_centroid.y << " " << bin_centroid.z << endl;
    bin_centroid.x += x_offset;
    bin_centroid.y += y_offset+0.00;
    bin_centroid.z += z_offset;
    cout << "Modified centroid: " << bin_centroid.x << " " << bin_centroid.y << " " << bin_centroid.z << endl;
    cout << "Obj centroid: " << obj_centroid.x << " " << obj_centroid.y << " " << obj_centroid.z << endl;
    std::vector<geometry_msgs::Point> waypoints;
    geometry_msgs::Point tmp = bin_centroid;
    tmp.y -= 0.15;
    waypoints.push_back(tmp);// first waypoint in front of bin centriod
    tmp = bin_centroid;
    tmp.z = obj_centroid.z + 0.065;// second waypoint at bin centroid with its height above the obj centroid
    waypoints.push_back(tmp);// second waypoint is bin centriod

    double interval_gap = 0.1;// going in steps

    vector< vector<double> > ik_jts;
    bool success = false;
    if(pk_dir == middle_front)// Middle front position
    {
//        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.03)/interval_gap;// num of steps
//        if(num_intervals < 0)
//            num_intervals = 0;
//        // generate rest all way points
//        tmp = obj_centroid;
//        tmp.y = bin_centroid.y;
//        for(int i=0; i<num_intervals; i++)
//        {
//            tmp.y += interval_gap*(i+1);
//            waypoints.push_back(tmp);
//        }

        tmp = obj_centroid;
        tmp.z += 0.065; // above the object centroid
        waypoints.push_back(tmp);
        tmp = obj_centroid;// at the object centroid
        waypoints.push_back(obj_centroid);
        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
        for(int i=0; i<waypoints.size(); i++)
        {
            vector<double> jts(7, 0.0);
            tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
            if(tmp_success)
            {
                ik_jts[i] = jts;
                continue;
            }
            else
            {
                success = false;
                ik_jts.resize(0);
                break;
            }
        }
        if(success)
        {
            cout << "Onward Joint angles: ";
            int b = 0;
            for(int i=0; i<ik_jts.size(); i++)
            {
                cout << i << ": [";
                for(int j=0; j<ik_jts[i].size(); j++)
                {
                    res.jts_reach.data.push_back(ik_jts[i][j]);
                    cout << res.jts_reach.data[b++] << " ";
                }
                cout << "]" << endl;
            }

            b=0;
            cout << "Return Joint angles: ";
            for(int i=ik_jts.size()-1; i>=0; i--)
            {
                cout << i << ": [";
                for(int j=0; j<ik_jts[i].size(); j++)
                {
                    res.jts_back.data.push_back(ik_jts[i][j]);
                    cout << res.jts_back.data[b++] << " ";
                }
                cout << "]" << endl;
            }
            return true;
        }
        else
            return false;
    }
    else if(pk_dir == left_side)// Left position
    {
//        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.03)/interval_gap;// num of steps
//        if(num_intervals < 0)
//            num_intervals = 0;
//        // generate rest all way points
//        tmp = bin_centroid;

//        for(int i=0; i<num_intervals; i++)
//        {
//            tmp.y += interval_gap*(i+1);
//            waypoints.push_back(tmp);
//        }

//        res.idx_bend.data.push_back(waypoints.size()-1);
        tmp = obj_centroid;
        tmp.x = bin_centroid.x; // a waypoint to turn left
        waypoints.push_back(tmp);
        tmp = obj_centroid;
        waypoints.push_back(tmp);

        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
        for(int i=0; i<=res.idx_bend.data[0]; i++) // first compute ik till the bending position
        {
            vector<double> jts(7, 0.0);
            tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
            if(tmp_success)
            {
                ik_jts[i] = jts;
                continue;
            }
            else
            {
                success = false;
                ik_jts.resize(0);
                break;
            }
        }
        if(success)// if successful till here then compute ik for bending position
        {
            for(int i=res.idx_bend.data[0]+1; i<waypoints.size(); i++)
            {
                vector<double> jts(7, 0.0);
                tmp_success = computeLeftIK(waypoints[i], bin_num, jts);
                if(tmp_success)
                {
                    ik_jts[i] = jts;
                    continue;
                }
                else
                {
                    success = false;
                    ik_jts.resize(0);
                    break;
                }
            }

            if(success)
            {
                cout << "Onward Joint angles: ";
                int b=0;
                for(int i=0; i<ik_jts.size(); i++)
                {
                    cout << i << ": [";
                    for(int j=0; j<ik_jts[i].size(); j++)
                    {
                        res.jts_reach.data.push_back(ik_jts[i][j]);
                        cout << res.jts_reach.data[b++] << " ";
                    }
                    cout << "]" << endl;
                }

// For returning out in case of left side compute ik keeping the arm in left direction
                cout << "Return Joint angles: ";
                b=0;
                for(int i=ik_jts.size()-1; i>=0; i--)
                {
                    cout << i << ": [";
                    for(int j=0; j<ik_jts[i].size(); j++)
                    {
                        res.jts_back.data.push_back(ik_jts[i][j]);
                        cout << res.jts_back.data[b++] << " ";
                    }
                    cout << "]" << endl;
                }
                return true;
            }
            else
                return false;
        }
    }
    else if(pk_dir == right_side)// Right position
    {
//        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.03)/interval_gap;// num of steps
//        if(num_intervals < 0)
//            num_intervals = 0;
//        // generate rest all way points
//        tmp = bin_centroid;

//        for(int i=0; i<num_intervals; i++)
//        {
//            tmp.y += interval_gap*(i+1);
//            waypoints.push_back(tmp);
//        }

//        res.idx_bend.data[0] = waypoints.size()-1;
        tmp = obj_centroid;
        tmp.x = bin_centroid.x; // a waypoint to turn right
        waypoints.push_back(tmp);
        tmp = obj_centroid;
        waypoints.push_back(tmp);

        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
        for(int i=0; i<=res.idx_bend.data[0]; i++) // first compute ik till the bending position
        {
            vector<double> jts(7, 0.0);
            tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
            if(tmp_success)
            {
                ik_jts[i] = jts;
                continue;
            }
            else
            {
                success = false;
                ik_jts.resize(0);
                break;
            }
        }
        if(success)// if successful till here then compute ik for bending position
        {
            for(int i=res.idx_bend.data[0]+1; i<waypoints.size(); i++)
            {
                vector<double> jts(7, 0.0);
                tmp_success = computeRightIK(waypoints[i], bin_num, jts);
                if(tmp_success)
                {
                    ik_jts[i] = jts;
                    continue;
                }
                else
                {
                    success = false;
                    ik_jts.resize(0);
                    break;
                }
            }

            if(success)
            {
                cout << "Onward Joint angles: ";
                int b=0;
                for(int i=0; i<ik_jts.size(); i++)
                {
                    cout << i << ": [";
                    for(int j=0; j<ik_jts[i].size(); j++)
                    {
                        res.jts_reach.data.push_back(ik_jts[i][j]);
                        cout << res.jts_reach.data[b++] << " ";
                    }
                    cout << "]" << endl;
                }

// For returning out in case of right side compute ik keeping the arm in right direction
                cout << "Return Joint angles: ";
                b=0;
                for(int i=ik_jts.size()-1; i>=0; i--)
                {
                    cout << i << ": [";
                    for(int j=0; j<ik_jts[i].size(); j++)
                    {
                        res.jts_back.data.push_back(ik_jts[i][j]);
                        cout << res.jts_back.data[b++] << " ";
                    }
                    cout << "]" << endl;
                }
                return true;
            }
            else
                return false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_rpt");
    ros::NodeHandle node;

    ros::ServiceServer service_traj_rpt;

    service_traj_rpt = node.advertiseService("/trajectory/rpt", getTrajCallback);

    ROS_INFO("Ready to get trajectory for rack pick task.");

    while(ros::ok())
        ros::spinOnce();
    return 0;

}
