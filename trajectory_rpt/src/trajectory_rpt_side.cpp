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

#define MAX_BC_GAP 0.05 // max distance of centroid from the bottom of the bin
#define MIN_SIDE_REQ 0.10 // min distance required from the side walls of the bin
#define MIN_GAP_REQ 0.1 // min height required above the object from the top of the bin
#define COS_THR 0.5
#define SIDE_ENTRY 0.08
#define TOP_ENTRY 0.05

#define X_RACK_OFFSET 0.0
#define Y_RACK_OFFSET -0.03
#define Z_RACK_OFFSET -0.08

#define OBJ_LEFT_SIDE_OFFSET 0.02
#define OBJ_RIGHT_SIDE_OFFSET -0.06

double obj_y_offset[40] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0};

double obj_x_offset[40] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double obj_z_offset[40] = {0.0, 0.04, 0.0, 0.0, 0.0, 0.02, 0.0, -0.025, -0.02, 0.0,
                           0.0, 0.015, 0.0, -0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

// look up table of how to pick objects
// -1: no picking
// 0: no preference
// 1: pick obj considering it to be either left or right side
// 2: pick obj considering it to be placed always at bottom
int obj_pick_lt[40] = {0, 2, 2, 2, 2,
                       1, 2, 1, 0, 2,
                       2, 2, 2, 1, 0,
                       0, 2, 2, 2, 0,
                       0, 0, 0, 2, 0,
                       2, -1, 0, 2, 2,
                       0, 2, 2, 2, -1,
                       2, 1, 2, 2, -1
                      };

enum pick_direction {middle_bottom, middle_front, left_side, right_side};
enum obj_position {psn_middle, psn_left, psn_right};

// refine centroid and pick direction taking into consideration of gap to be maintained
// function to refine the eef for picking the object
// robot rotated by 90deg in z-direction
// centroid: centroid of the object
// normal_vect: normal vector at the centroid
// corners: 4 bin corner points
// refined_centroid: refined centroid to be processed and sent back
// obj_corners: 4 extreme points of the object
// pdir: picking direction
bool refineeef1(geometry_msgs::Point &centroid, geometry_msgs::Point &normal_vect,
                vector<geometry_msgs::Point> &corners, geometry_msgs::Point &refined_centroid,
                vector<geometry_msgs::Point> &obj_corners, pick_direction &pdir, int &obj_id)
{
    tf::Vector3 eef_psn(centroid.x, centroid.y, centroid.z);
    tf::Vector3 eef_normal(normal_vect.x, normal_vect.y, normal_vect.z);

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

    cout << "Bin Left: " << lc.x << " " << lc.y << " " << lc.z << endl;
    cout << "Bin top : " << tc.x << " " << tc.y << " " << tc.z << endl;
    cout << "Bin Rigt: " << rc.x << " " << rc.y << " " << rc.z << endl;
    cout << "Bin Bott: " << bc.x << " " << bc.y << " " << bc.z << endl;

    double z_max, x_min, x_max;
    z_max = 0;
    for(int i=0; i<3; i++)
    {
        if(obj_corners[i].z > z_max)
            z_max = obj_corners[i].z;
    }
    if(centroid.z > z_max)
        z_max = centroid.z;

    x_max = -100;
    for(int i=0; i<3; i++)
    {
        if(obj_corners[i].x > x_max)
            x_max = obj_corners[i].x;
    }
    if(centroid.x > x_max)
        x_max = centroid.x;

    x_min = 100;
    for(int i=0; i<3; i++)
    {
        if(obj_corners[i].x < x_min)
            x_min = obj_corners[i].x;
    }
    if(centroid.x < x_min)
        x_min = centroid.x;

    cout << "z_max: " << z_max << " x_min: " << x_min << " x_max: " << x_max << endl;
    cout << "Obj id: " << obj_id << endl;
    cout << "Object: " << model_names[obj_id-1] << " ";
    // find the picking direction
    {
        if(obj_pick_lt[obj_id-1] == 0)// if 0 dynamically find picking direction
        {
            if(fabs(tc.z - z_max) > MIN_GAP_REQ+0.02)
            {
                pdir = middle_bottom;
                cout << "Object is: middle" << endl;
            }
            else if(fabs(bc.z - centroid.z) < MAX_BC_GAP)
            {
                pdir = middle_bottom;
                cout << "Object is: middle" << endl;
            }
            else if(fabs(lc.x - x_min) > MIN_SIDE_REQ)// if object's left pt is at distance > 8cm from left side of bin
            {
                pdir = right_side;// consider object to be at right side
                cout << "Object is: right side" << endl;
            }

            else if(fabs(x_max - rc.x) > MIN_SIDE_REQ)// if object's right pt is at distance > 8cm from right side of bin
            {
                pdir = left_side;// consider object to be at left side
                cout << "Object is: left side" << endl;
            }

            else if(fabs(tc.z - z_max) > MIN_GAP_REQ)// if object's top point is at distance > 8cm from top of bin
            {
                pdir = middle_bottom;
                cout << "Object is: middle" << endl;
            }
            else
            {
                // choose the one that has the maximum gap
                double gap_top=fabs(tc.z - z_max);
                double gap_right=fabs(lc.x - x_min);
                double gap_left=fabs(x_max - rc.x);
                if(gap_top > gap_right && gap_top > gap_left)
                {
                    pdir = right_side;
                    cout << "Object is: right side" << endl;
                }
                else if(gap_right > gap_top && gap_right > gap_left)
                {
                    pdir = left_side;
                    cout << "Object is: left side" << endl;
                }
                else
                {
                    pdir=middle_bottom;
                    cout << "Object is: middle" << endl;
                }
//                cout << "Does not have minimum gap requirement from the top" << endl;
//                return false;
            }
        }
        else if(obj_pick_lt[obj_id-1] == 1)// if 1 find left or right side
        {
            if(fabs(lc.x - x_min) > MIN_SIDE_REQ)// if object's left pt is at distance > 8cm from left side of bin
            {
                pdir = right_side;// consider object to be at right side
                cout << "Object is: right side" << endl;
            }
            else if(fabs(x_max - rc.x) > MIN_SIDE_REQ)// if object's right pt is at distance > 8cm from right side of bin
            {
                pdir = left_side;// consider object to be at left side
                cout << "Object is: left side" << endl;
            }
            else
            {
                // choose the one that has the maximum gap
                double gap_top=fabs(tc.z - z_max);
                double gap_right=fabs(lc.x - x_min);
                double gap_left=fabs(x_max - rc.x);
                if(gap_right > gap_top && gap_right > gap_left)
                {
                    pdir = right_side;
                    cout << "Object is: right side" << endl;
                }
                else
                {
                    pdir = left_side;
                    cout << "Object is: left side" << endl;
                }
            }
        }
        else if(obj_pick_lt[obj_id-1] == 2)// if 2 object will be placed at bottom
        {
            pdir = middle_bottom;
            cout << "Object is: middle" << endl;
        }
        else if(obj_pick_lt[obj_id-1] == -1)// return false, object cannot be picked
        {
            cout << "No picking method " << endl;
            return false;
        }

    }

    // refine the eef position so that the arm won't collide with the bin
    refined_centroid.x = eef_psn.x();
    refined_centroid.y = eef_psn.y() + obj_y_offset[obj_id-1];
    refined_centroid.z = eef_psn.z();
    if(pdir == middle_bottom)
    {
        tf::Vector3 robot_base_ny(0,-1,0);
        if(eef_normal.dot(robot_base_ny) > COS_THR) // if normal is along -ve y-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.035+ obj_y_offset[obj_id-1];
        refined_centroid.x += 0.02;
        refined_centroid.z += obj_z_offset[obj_id-1];
    }

    else if(pdir == left_side)
    {
        tf::Vector3 robot_base_x(1,0,0);
        if(eef_normal.dot(robot_base_x) < COS_THR)// if normal is not along x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02+ obj_y_offset[obj_id-1];
        refined_centroid.x += 0.01;
        refined_centroid.z += obj_z_offset[obj_id-1];
    }
    else if(pdir == right_side)
    {
        tf::Vector3 robot_base_nx(-1,0,0);
        if(eef_normal.dot(robot_base_nx) < COS_THR)// if normal is not along -ve x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02+ obj_y_offset[obj_id-1];
        refined_centroid.z += obj_z_offset[obj_id-1];
    }
    return true;
}

// could take into consideration of gap to be maintained
// function to refine the eef for picking the object
// robot rotated by 90deg in z-direction
// centroid: centroid of the object
// normal_vect: normal vector at the centroid
// corners: 4 bin corner points
// refined_centroid: refined centroid to be processed and sent back
// pdir: picking direction
void refineeef2(geometry_msgs::Point &centroid, geometry_msgs::Point &normal_vect,
                vector<geometry_msgs::Point> &corners, geometry_msgs::Point &refined_centroid,
                pick_direction &pdir)
{
    tf::Vector3 eef_psn(centroid.x, centroid.y, centroid.z);
    tf::Vector3 eef_normal(normal_vect.x, normal_vect.y, normal_vect.z);

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
        if(eef_normal.dot(robot_base_ny) > MIN_GAP_REQ) // if normal is along -ve y-axis of robot base add 2cm along y-axis
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
        if(eef_normal.dot(robot_base_x) < MIN_GAP_REQ)// if normal is not along x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02;
    }
    else if(pdir == right_side)
    {
        tf::Vector3 robot_base_nx(-1,0,0);
        if(eef_normal.dot(robot_base_nx) < MIN_GAP_REQ)// if normal is not along -ve x-axis of robot base add 2cm along y-axis
            refined_centroid.y = eef_psn.y()+0.02;
    }
    return;
}


// add obj_id into the service call
bool getTrajCallback(trajectory_rpt::TrajRpt::Request &req, trajectory_rpt::TrajRpt::Response &res)
{
    std::cout << "Reached service for trajectory planning rpt" << std::endl;

    double x_offset = X_RACK_OFFSET, y_offset = Y_RACK_OFFSET, z_offset = Z_RACK_OFFSET;

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

    // lc: left corners avg, rc: right corners avg, tc, top corners avg, bc, bottom corners avg
    geometry_msgs::Point lc, rc, tc, bc;
    lc.x = (bin_corners[2].x + bin_corners[0].x)/2;
    lc.y = (bin_corners[2].y + bin_corners[0].y)/2;
    lc.z = (bin_corners[2].z + bin_corners[0].z)/2;

    rc.x = (bin_corners[1].x + bin_corners[3].x)/2;
    rc.y = (bin_corners[1].y + bin_corners[3].y)/2;
    rc.z = (bin_corners[1].z + bin_corners[3].z)/2;

    tc.x = (bin_corners[0].x + bin_corners[1].x)/2;
    tc.y = (bin_corners[0].y + bin_corners[1].y)/2;
    tc.z = (bin_corners[0].z + bin_corners[1].z)/2;

    bc.x = (bin_corners[2].x + bin_corners[3].x)/2;
    bc.y = (bin_corners[2].y + bin_corners[3].y)/2;
    bc.z = (bin_corners[2].z + bin_corners[3].z)/2;

    vector<geometry_msgs::Point> obj_corners(3);
    idx=0;
    obj_corners[0] = req.obj_left;
    obj_corners[1] = req.obj_top;
    obj_corners[2] = req.obj_right;

    cout << "Received Object corners:" << endl;
    for(int i=0; i<3; i++)
        cout << "[" << obj_corners[i].x << " " << obj_corners[i].y << " " << obj_corners[i].z << "]" << endl;

    // object id required to give offset to the object centroid
    int obj_id;
    obj_id = req.obj_id.data;

    cout << "Obj ID: " << req.obj_id.data << endl;
    cout << "Received IK call for object: " << model_names[req.obj_id.data-1] << endl;

    int bin_num = req.bin_num.data;
    geometry_msgs::Point obj_centroid;
    obj_centroid.x = req.obj_centroid.x;
    obj_centroid.y = req.obj_centroid.y;
    obj_centroid.z = req.obj_centroid.z;
    cout << "Received Obj centroid: " << obj_centroid.x << " " << obj_centroid.y << " " << obj_centroid.z << endl;

    if(fabs(obj_centroid.x - 0.0) < 0.001 && fabs(obj_centroid.y - 0.0) < 0.001 && fabs(obj_centroid.z - 0.0) < 0.001)
    {
        cout << "Received object centroid at (0.0, 0.0, 0.0)" << endl;
        cout << "Cannot compute trajectory" << endl;
        return false;
    }

    geometry_msgs::Point obj_normal;
    obj_normal.x = req.obj_normal.x;
    obj_normal.y = req.obj_normal.y;
    obj_normal.z = req.obj_normal.z;

    pick_direction pk_dir;
#if(DEFAULT_MIDDLE_FRONT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += 0.00;
    obj_centroid.y += 0.08;
    obj_centroid.z += 0.00;
    pk_dir = middle_front;
    for(int i=0; i<3; i++)
        obj_corners[i] = obj_centroid;
    obj_id = 1;
#elif(DEFAULT_LEFT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += 0.00 - 0.06;
    obj_centroid.y += 0.00+0.08;
    obj_centroid.z += 0.00;
    pk_dir = left_side;
    for(int i=0; i<3; i++)
        obj_corners[i] = obj_centroid;
    obj_id = 1;
#elif(DEFAULT_RIGHT_POSITION)
    obj_centroid = bin_centroid;
    obj_centroid.x += 0.00 + 0.06;
    obj_centroid.y += 0.00+0.08;
    obj_centroid.z += 0.00;
    pk_dir = right_side;
    for(int i=0; i<3; i++)
        obj_corners[i] = obj_centroid;
    obj_id = 1;
#else
    // Obtain object placing position and picking direction
//    refineeef2(obj_centroid, obj_normal, bin_corners, obj_centroid, pk_dir);
    if(!refineeef1(obj_centroid, obj_normal, bin_corners, obj_centroid, obj_corners, pk_dir, obj_id))
        return false;
//    pk_dir = middle_front;
#endif

    cout << "Bin centroid: " << bin_centroid.x << " " << bin_centroid.y << " " << bin_centroid.z << endl;
    bin_centroid.x += x_offset;
    bin_centroid.y += y_offset;
    bin_centroid.z += z_offset;
    // update obj_corners with the offset
    for(int i=0; i<3; i++)
    {
        obj_corners[i].x += x_offset;
        obj_corners[i].y += y_offset;
        obj_corners[i].z += z_offset;
    }
    cout << "Modified bin centroid: " << bin_centroid.x << " " << bin_centroid.y << " " << bin_centroid.z << endl;
    cout << "Modified Object corners:" << endl;
    for(int i=0; i<3; i++)
        cout << "[" << obj_corners[i].x << " " << obj_corners[i].y << " " << obj_corners[i].z << "]" << endl;

    obj_centroid.x += x_offset;
    obj_centroid.y += y_offset;
    obj_centroid.z += z_offset;
    cout << "Refined Obj centroid: " << obj_centroid.x << " " << obj_centroid.y << " " << obj_centroid.z << endl;

    std::vector<geometry_msgs::Point> waypoints;
    geometry_msgs::Point tmp = bin_centroid;// Move towards bin in first 3 steps
    tmp.y -= 0.15;
    tmp.z += 0.065;
    waypoints.push_back(tmp);

    tmp = bin_centroid;
    tmp.y -= 0.15;
    tmp.z += 0.065;
    double side_x;
    double z_max, x_min, x_max;
    if(pk_dir == middle_bottom)// if object has to be picked from bottom middle
    {
        z_max = 0;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].z > z_max)
                z_max = obj_corners[i].z;
        }
        if(obj_centroid.z > z_max)
            z_max = obj_centroid.z;

        x_max = -100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x > x_max)
                x_max = obj_corners[i].x;
        }
        if(obj_centroid.x > x_max)
            x_max = obj_centroid.x;

        x_min = 100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x < x_min)
                x_min = obj_corners[i].x;
        }
        if(obj_centroid.x < x_min)
            x_min = obj_centroid.x;

        tmp.z = z_max + (double) TOP_ENTRY;// second waypoint at bin centroid with its height above the obj centroid
    }
    else if(pk_dir == left_side)
    {
        // modify x if object is on left side
        obj_centroid.x += OBJ_LEFT_SIDE_OFFSET;
        for(int i=0; i<3; i++)
            obj_corners[i].x += OBJ_LEFT_SIDE_OFFSET;

        z_max = 0;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].z > z_max)
                z_max = obj_corners[i].z;
        }
        if(obj_centroid.z > z_max)
            z_max = obj_centroid.z;

        x_max = -100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x > x_max)
                x_max = obj_corners[i].x;
        }
        if(obj_centroid.x > x_max)
            x_max = obj_centroid.x;

        x_min = 100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x < x_min)
                x_min = obj_corners[i].x;
        }
        if(obj_centroid.x < x_min)
            x_min = obj_centroid.x;

        side_x = x_max+(double) SIDE_ENTRY;
//        side_x = rc.x-0.03;// so that it won't collide with right side wall ever
        tmp.x = side_x+0.01;
    }
    else if(pk_dir == right_side)
    {
        // modify x if object is on right side
        obj_centroid.x += OBJ_RIGHT_SIDE_OFFSET;
        for(int i=0; i<3; i++)
            obj_corners[i].x += OBJ_RIGHT_SIDE_OFFSET;

        z_max = 0;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].z > z_max)
                z_max = obj_corners[i].z;
        }
        if(obj_centroid.z > z_max)
            z_max = obj_centroid.z;

        x_max = -100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x > x_max)
                x_max = obj_corners[i].x;
        }
        if(obj_centroid.x > x_max)
            x_max = obj_centroid.x;

        x_min = 100;
        for(int i=0; i<3; i++)
        {
            if(obj_corners[i].x < x_min)
                x_min = obj_corners[i].x;
        }
        if(obj_centroid.x < x_min)
            x_min = obj_centroid.x;

        side_x = x_min-(double) SIDE_ENTRY;
//        side_x = lc.x + 0.03;// so that it won't collide with left side wall ever
        tmp.x = side_x;
    }

    cout << "z_max: " << z_max << " x_min: " << x_min << " x_max: " << x_max << endl;

    double interval_gap = 0.025;// going in steps
    int n_steps = fabs(bin_centroid.y - tmp.y)/interval_gap;// num of steps
    cout << "Will reach bin centroid in " << n_steps+1 << endl;
    for(int j=0; j<interval_gap; j++)
    {
        tmp.y += interval_gap;// 10cm far away from bin
        if(z_max<bin_centroid.z)
            tmp.z = bin_centroid.z + 0.085;
//        else
//            tmp.z = bin_centroid.z + 0.085;
        waypoints.push_back(tmp);
    }
//    tmp.y += 0.05;// 5cm far away from bin
//    waypoints.push_back(tmp);
//    tmp.y += 0.05;// 0cm far away from bin
//    waypoints.push_back(tmp);

    cout << "Interval gap: " << interval_gap << endl;

    vector< vector<double> > ik_jts;
    bool success = false;
    if(pk_dir == middle_bottom)// Middle front position
    {
        cout << "Object to picked from bottom" << endl;
        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.01)/interval_gap;// num of steps
        if(num_intervals < 0)
            num_intervals = 0;
        // generate rest all way points
        tmp = obj_centroid;
        tmp.y = bin_centroid.y;
        if(z_max<bin_centroid.z)
            tmp.z = bin_centroid.z + 0.11;
        else
            tmp.z = z_max+(double) TOP_ENTRY;
        for(int i=0; i<num_intervals; i++)
        {
            tmp.y += interval_gap;
            waypoints.push_back(tmp);
        }

        tmp = obj_centroid;
        tmp.z = z_max+(double) TOP_ENTRY; // above the object centroid with keeping gap of 7.5cm from the top
        waypoints.push_back(tmp);
        tmp = obj_centroid;// at the object centroid
        waypoints.push_back(obj_centroid);
        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
        for(int i=0; i<waypoints.size(); i++)
        {
            cout << "Waypoint " << i+1 << ": [" << waypoints[i].x << " " << waypoints[i].y << " " << waypoints[i].z << "]" << endl;
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
            cout << "\nOnward Joint angles: \n";
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
            for(int i=ik_jts.size()-2; i>=0; i--)
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
        cout << "Object to picked from left side" << endl;
        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.03)/interval_gap;// num of steps
        if(num_intervals < 0)
            num_intervals = 0;
        // generate rest all way points
        tmp = bin_centroid;

        for(int i=0; i<num_intervals; i++)
        {
            tmp.x = side_x;
            tmp.y += interval_gap;
            tmp.z += 0.045;
            waypoints.push_back(tmp);
        }

        int rotate_p = waypoints.size()-1;
//        res.idx_bend.data.push_back(waypoints.size()-1);
        tmp = obj_centroid;
//        tmp.x = bin_centroid.x; // a waypoint to turn left
        tmp.x = side_x; // a waypoint to turn left
        waypoints.push_back(tmp);
        tmp = obj_centroid;
        waypoints.push_back(tmp);

        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
//        for(int i=0; i<=res.idx_bend.data[0]; i++) // first compute ik till the bending position
        for(int i=0; i<=rotate_p; i++) // first compute ik till the bending position
        {
            cout << "Waypoint " << i+1 << ": [" << waypoints[i].x << " " << waypoints[i].y << " " << waypoints[i].z << "]" << endl;
            vector<double> jts(7, 0.0);
//            if(i==0)
            tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
//            else
//                tmp_success = computeLeftIK(waypoints[i], bin_num, jts);

            if(tmp_success)
            {
                IkReal *r_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
                for(int r=0; r<NO_JOINTS; r++)
                    r_jts[r]=jts[r];
                IkReal eerot[9], eetrans[3];
                ComputeFk(r_jts, eetrans, eerot);
                tf::Vector3 nx(-1.0, 0.0, 0.0);
                tf::Vector3 x_vect(eerot[0],eerot[3],eerot[6]);
                double d_prod = x_vect.dot(nx);
                double rad = acos(d_prod);

                jts[6] += rad;
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
//            for(int i=res.idx_bend.data[0]+1; i<waypoints.size(); i++)
            for(int i=rotate_p+1; i<waypoints.size(); i++)
            {
                cout << "Waypoint " << i+1 << ": [" << waypoints[i].x << " " << waypoints[i].y << " " << waypoints[i].z << "]" << endl;
                vector<double> jts(7, 0.0);
//                tmp_success = computeLeftIK(waypoints[i], bin_num, jts);
                tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
                if(tmp_success)
                {
                    IkReal *r_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
                    for(int r=0; r<NO_JOINTS; r++)
                        r_jts[r]=jts[r];
                    IkReal eerot[9], eetrans[3];
                    ComputeFk(r_jts, eetrans, eerot);
                    tf::Vector3 nx(-1.0, 0.0, 0.0);
                    tf::Vector3 x_vect(eerot[0],eerot[3],eerot[6]);
                    double d_prod = x_vect.dot(nx);
                    double rad = acos(d_prod);

                    jts[6] += rad;
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

            // push all the trajectory joint angles
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
                for(int i=ik_jts.size()-2; i>=0; i--)
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
        cout << "Object to picked from right side" << endl;
        int num_intervals = (obj_centroid.y - bin_centroid.y - 0.03)/interval_gap;// num of steps
        if(num_intervals < 0)
            num_intervals = 0;
        // generate rest all way points
        tmp = bin_centroid;

        for(int i=0; i<num_intervals; i++)
        {
            tmp.x = side_x;
            tmp.y += interval_gap;
            tmp.z += 0.045;
            waypoints.push_back(tmp);
        }

        int rotate_p = waypoints.size()-1;
//        res.idx_bend.data[0] = waypoints.size()-1;
        tmp = obj_centroid;
//        tmp.x = bin_centroid.x; // a waypoint to turn right
        tmp.x = side_x; // a waypoint to turn right
        waypoints.push_back(tmp);
        tmp = obj_centroid;
        waypoints.push_back(tmp);

        ik_jts = vector<vector<double> > (waypoints.size(), vector<double> (7));

        // compute IK for all waypoints
        bool tmp_success = false;
        success = true;
//        for(int i=0; i<=res.idx_bend.data[0]; i++) // first compute ik till the bending position
        for(int i=0; i<=rotate_p; i++) // first compute ik till the bending position
        {
            cout << "Waypoint " << i+1 << ": [" << waypoints[i].x << " " << waypoints[i].y << " " << waypoints[i].z << "]" << endl;
            vector<double> jts(7, 0.0);
//            if(i==0)
            tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
//            else
//                tmp_success = computeRightIK(waypoints[i], bin_num, jts);
            if(tmp_success)
            {
                IkReal *r_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
//                cout << "Computing forward Kinematics: \n[";

                for(int r=0; r<NO_JOINTS; r++)
                {
                    r_jts[r]=jts[r];
                    cout << r_jts[r] << " ";
                }
                cout << "]" << endl;
                IkReal eerot[9], eetrans[3];


                ComputeFk(r_jts, eetrans, eerot);
//                cout << "X_vector: [" << eerot[0] << " " << eerot[3] << " " << eerot[6] << "]" << endl;
                tf::Vector3 x(1.0, 0.0, 0.0);
                tf::Vector3 x_vect(eerot[0],eerot[3],eerot[6]);
                double d_prod = x_vect.dot(x);
                double rad = acos(d_prod);
//                cout << "d_prod: " << d_prod  << " rad: " << rad << endl;

//                cout << "jts[6] " << jts[6] << " ";
                jts[6] -= rad;
//                cout << jts[6] << endl;

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
//            for(int i=res.idx_bend.data[0]+1; i<waypoints.size(); i++)
            for(int i=rotate_p+1; i<waypoints.size(); i++)
            {
                cout << "Waypoint " << i+1 << ": [" << waypoints[i].x << " " << waypoints[i].y << " " << waypoints[i].z << "]" << endl;
                vector<double> jts(7, 0.0);
                tmp_success = computeMiddleIK(waypoints[i], bin_num, jts);
//                tmp_success = computeRightIK(waypoints[i], bin_num, jts);
                if(tmp_success)
                {
                    IkReal *r_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
                    for(int r=0; r<NO_JOINTS; r++)
                        r_jts[r]=jts[r];
                    IkReal eerot[9], eetrans[3];
                    ComputeFk(r_jts, eetrans, eerot);
                    tf::Vector3 x(1.0, 0.0, 0.0);
                    tf::Vector3 x_vect(eerot[0],eerot[3],eerot[6]);
                    double d_prod = x_vect.dot(x);
                    double rad = acos(d_prod);

                    jts[6] -= rad;
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
                for(int i=ik_jts.size()-2; i>=0; i--)
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
