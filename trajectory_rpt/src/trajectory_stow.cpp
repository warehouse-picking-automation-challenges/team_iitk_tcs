#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <trajectory_rpt/trajectory_stow.h>
#include <trajectory_rpt/TrajStow.h>
#include <trajectory_rpt/ContinuousTrajGen.h>

#define DOT_TH 0.85 // for cos(30) to cos(0)

//double x_offset = 0.03, y_offset = 0.08, z_offset = -0.1;
double x_offset = 0.00, y_offset = 0.00, z_offset = -0.085;
double x_rack_offset = 0.035, y_rack_offset = -0.0, z_rack_offset = 0.01;
double tote_retract_psn[NO_JOINTS] = {-1.58285, 0.400974, 0.0959679, 2.08, -0.161621, 0.339057, -0.181756};
double tote_above_psn[NO_JOINTS] = {-1.54552, 0.671151, 0.0395913, 1.82501, -0.135291, 0.56955, 0.435844};

double tote_to_rack[][NO_JOINTS] = {{}};

#define WAM_X 0.22
#define WAM_Y 0.14
#define WAM_Z 0.406

#define RIGHT_EXTN_DISTANCE 0.005
#define LEFT_EXTN_DISTANCE 0.00

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

// look up table whether to pick object from sides or not
// -1: no picking
// 0: definitely no side picking
// 1: side picking can be done
int obj_pick_lt[40] = {0, 0, 0, 1, 1,
                       0, 0, 0, 0, 1,
                       0, 0, 0, 0, 1,
                       1, 0, 0, 0, 1,
                       0, 1, 1, 0, 0,
                       1, -1, 1, 0, 1,
                       1, 0, 0, 1, -1,
                       0, 1, 1, 0, -1
                      };

ros::ServiceClient continuousTrajService;

bool getTrajCallback(trajectory_rpt::TrajStow::Request &req, trajectory_rpt::TrajStow::Response &res)
{
    cout << "Entered stow trajectory generation service" << endl;
    bool success = false;



    if(isnan(req.obj_normal.x) || isnan(req.obj_centroid.x) )//| isnan(req.obj_axis.x))
    {
        cout << "Found NaN" << endl;
        return false;
    }
    else if(obj_pick_lt[req.obj_id.data-1] == -1)
    {
        cout << model_names[req.obj_id.data-1].c_str() << " object cannot be picked" << endl;
        return false;
    }

//    else if(isnan(req.gripping_axis.x) || isnan(req.gripping_centroid.x) || isnan(req.gripping_normal.x))
//    {
//        cout << "Found NaN" << endl;
//        return false;
//    }


    // if flag_grip is true then try to get the inverse kinematics for gripping the object
//    if(req.flag_grip.data)
//    {
//        cout << "Finding trajectory for gripper mode" << endl;

//        std::vector<geometry_msgs::Point> waypoints;
//        geometry_msgs::Point tmp;
//        tf::Vector3 grip_normal(req.gripping_normal.x, req.gripping_normal.y, req.gripping_normal.z);

//        grip_normal.normalize();
//        if(grip_normal.x() < 0)
//        {
//            cout << "Reversed normal vector" << endl;
//            grip_normal *= -1;
//        }
//        // Find a way point 5cm far from gripper centroid
//        grip_normal *= 0.15;


//        tmp.x = req.gripping_centroid.x + grip_normal.x() + x_offset;
//        tmp.y = req.gripping_centroid.y + grip_normal.y() + y_offset;
//        tmp.z = req.gripping_centroid.z + grip_normal.z() + z_offset;

//        waypoints.push_back(tmp);
//        // Push the gripping centroid
//        tmp.x = req.gripping_centroid.x + x_offset;
//        tmp.y = req.gripping_centroid.y + y_offset;
//        tmp.z = req.gripping_centroid.z + z_offset;
//        waypoints.push_back(tmp);

//        cout << "Gripping centroid: " << req.gripping_centroid.x << " " << req.gripping_centroid.y << " " << req.gripping_centroid.z << endl;
//        cout << "Gripping axis: " << req.gripping_axis.x << " " << req.gripping_axis.y << " " << req.gripping_axis.z << endl;
//        cout << "Gripping normal: " << req.gripping_normal.x << " " << req.gripping_normal.y << " " << req.gripping_normal.z << endl;

//        // Call function to compute inverse kinematics
//        tf::Vector3 grip_axis(req.gripping_axis.x, req.gripping_axis.y, req.gripping_axis.z);
//        vector<vector<double> > trajectory;
//        success = computeGripperIK(waypoints, grip_axis, grip_normal, trajectory);

//        if(success)
//        {
//            // if success copy the jt trajectory angles values
//            res.flag_grip.data = true;
//            for(int i=0; i<trajectory.size(); i++)
//            {
//                for(int j=0; j<trajectory[i].size(); j++)
//                    res.jts_reach.data.push_back(trajectory[i][j]);
//            }

//            for(int i=trajectory.size()-2; i>=0; i--)
//            {
//                for(int j=0; j<trajectory[i].size(); j++)
//                    res.jts_back.data.push_back(trajectory[i][j]);
//            }
//            for(int j=0; j<NO_JOINTS; j++)
//                res.jts_back.data.push_back(tote_retract_psn[j]);

//            return true;
//        }
//        else
//        {
//            cout << "Failed to compute IK for gripper mode" << endl;
//            return false;

//            // Try with some default orientation for gripping
//            cout << "Going to try gripping with some default joint angles mode" << endl;
//            success = computeGripperDefaultIK(waypoints, trajectory);
//            if(success)
//            {
//                res.flag_grip.data = true;
//                for(int i=0; i<trajectory.size(); i++)
//                {
//                    for(int j=0; j<trajectory[i].size(); j++)
//                        res.jts_reach.data.push_back(trajectory[i][j]);
//                }

//                for(int i=trajectory.size()-2; i>=0; i--)
//                {
//                    for(int j=0; j<trajectory[i].size(); j++)
//                        res.jts_back.data.push_back(trajectory[i][j]);
//                }
//                for(int j=0; j<NO_JOINTS; j++)
//                    res.jts_back.data.push_back(tote_retract_psn[j]);
//                return true;
//            }
//            else
//                return false;
//            cout << "Going for suction with bottom mode" << endl;
//        }
//    }

    // go for suction ik planning in case gripper planning fails
    if(!success)
    {
        cout << "Finding trajectory for suction mode" << endl;
        cout << "object centroid: " << req.obj_centroid.x << " " << req.obj_centroid.y << " " << req.obj_centroid.z << endl;
//        cout << "object axis: " << req.obj_axis.x << " " << req.obj_axis.y << " " << req.obj_axis.z << endl;
        cout << "object normal: " << req.obj_normal.x << " " << req.obj_normal.y << " " << req.obj_normal.z << endl;

        vector<geometry_msgs::Point> waypoints;
        vector<geometry_msgs::Point> waypoints_rack;
        geometry_msgs::Point tmp;
        tf::Vector3 obj_normal(req.obj_normal.x, req.obj_normal.y, req.obj_normal.z);
        obj_normal.normalize();
        int suction_orientation = req.obj_psn_suction.data;

        if(obj_pick_lt[req.obj_id.data-1] == 1)// if object can be picked from sides
        {
            // dot product with z-axis
            double dot_pro = fabs(obj_normal.z());
            if(dot_pro > DOT_TH)// if normal is at angle less than 30 degress from z-axis of robot then consider to pick it from bottom
            {
                cout << suction_orientation << " changed to tote_bottom for normal angles " << acos(dot_pro) << endl;
                suction_orientation = TOTE_BOTTOM;
            }
        }
        else
        {
            cout << suction_orientation << " changed to tote_bottom for object " << model_names[req.obj_id.data-1] << endl;
            suction_orientation = TOTE_BOTTOM;
        }


        if(suction_orientation == TOTE_LEFT)
        {
            cout << "Tote left mode" << endl;
            // get a trj point 6cm far and 5 cm below from the object
            double x_far = -0.06, y_far = 0.00, z_abv = 0.02;
            tmp.x = req.obj_centroid.x + x_far + x_offset;
            tmp.y = req.obj_centroid.y + y_far + y_offset;
            tmp.z = req.obj_centroid.z + z_abv + z_offset;
            waypoints.push_back(tmp);

            // second point at 5cm below the object centroid
            tmp.x = req.obj_centroid.x + x_offset;
            tmp.y = req.obj_centroid.y + y_offset;
            tmp.z = req.obj_centroid.z + z_offset + z_abv;
            waypoints.push_back(tmp);
        }
        else if(suction_orientation == TOTE_INWARDS)
        {
            cout << "Tote inwards mode" << endl;
            // get a trj point 6cm far and 5 cm below from the object
            double x_far = 0.00, y_far = -0.06, z_abv = 0.02;
            tmp.x = req.obj_centroid.x + x_far + x_offset;
            tmp.y = req.obj_centroid.y + y_far + y_offset;
            tmp.z = req.obj_centroid.z + z_abv + z_offset;
            waypoints.push_back(tmp);

            // second point at 5cm below the object centroid
            tmp.x = req.obj_centroid.x + x_offset;
            tmp.y = req.obj_centroid.y + y_offset;
            tmp.z = req.obj_centroid.z + z_offset + z_abv;
            waypoints.push_back(tmp);
        }
        else if(suction_orientation == TOTE_RIGHT)
        {
            cout << "Tote right mode" << endl;
            // get a trj point 6cm far and 5 cm below from the object
            double x_far = 0.06, y_far = 0.00, z_abv = 0.02;
            tmp.x = req.obj_centroid.x + x_far + x_offset;
            tmp.y = req.obj_centroid.y + y_far + y_offset;
            tmp.z = req.obj_centroid.z + z_abv + z_offset;
            waypoints.push_back(tmp);

            // second point at 5cm below the object centroid
            tmp.x = req.obj_centroid.x + x_offset;
            tmp.y = req.obj_centroid.y + y_offset;
            tmp.z = req.obj_centroid.z + z_offset + z_abv;
            waypoints.push_back(tmp);
        }
        else if(suction_orientation == TOTE_OUTWARDS)
        {
            cout << "Tote outwards mode" << endl;
            // get a trj point 6cm far and 5 cm below from the object
            double x_far = 0.00, y_far = 0.06, z_abv = 0.02;
            tmp.x = req.obj_centroid.x + x_far + x_offset;
            tmp.y = req.obj_centroid.y + y_far + y_offset;
            tmp.z = req.obj_centroid.z + z_abv + z_offset;
            waypoints.push_back(tmp);

            // second point at 5cm below the object centroid
            tmp.x = req.obj_centroid.x + x_offset;
            tmp.y = req.obj_centroid.y + y_offset;
            tmp.z = req.obj_centroid.z + z_offset + z_abv;
            waypoints.push_back(tmp);
        }
        else if(suction_orientation == TOTE_BOTTOM)
        {
            cout << "Tote bottom mode" << endl;
            // get a trj point 6cm above the object
            double x_far = 0.00, y_far = 0.00, z_abv = 0.06;
            tmp.x = req.obj_centroid.x + x_far + x_offset;
            tmp.y = req.obj_centroid.y + y_far + y_offset;
            tmp.z = req.obj_centroid.z + z_abv + z_offset; // 6cm is added so as reach above the object
            waypoints.push_back(tmp);

            // second point at the object
            tmp.x = req.obj_centroid.x + x_offset;
            tmp.y = req.obj_centroid.y + y_offset;
            tmp.z = req.obj_centroid.z + z_offset;
            waypoints.push_back(tmp);
        }

        // generate waypoints to go inside rack
        // first waypoint infront of bin
        tmp.x = req.bin_centroid.data[0] + x_rack_offset;
        tmp.y = req.bin_centroid.data[1] - 0.16 + y_rack_offset;
        tmp.z = req.bin_centroid.data[2] + z_rack_offset;
        waypoints_rack.push_back(tmp);

        // third waypoint 12cm inside bin
        tmp.y = req.bin_centroid.data[1] - 0.1 + y_rack_offset;
        waypoints_rack.push_back(tmp);

        // In case of right column bins
        if(req.bin_num.data == 5 || req.bin_num.data == 8 || req.bin_num.data == 11)
            tmp.x += RIGHT_EXTN_DISTANCE;

        // In case of left column bins
        if(req.bin_num.data == 3 || req.bin_num.data == 6 || req.bin_num.data == 9)
            tmp.x += LEFT_EXTN_DISTANCE;

        waypoints_rack.push_back(tmp);

        // second waypoint at bin centroid
        tmp.y = req.bin_centroid.data[1] + y_rack_offset;
        waypoints_rack.push_back(tmp);

        // third waypoint 18cm inside bin
        tmp.y = req.bin_centroid.data[1] + 0.18 + y_rack_offset;


//        in case of chenille stems and bubble mailer, 5cm more for last point in bin
        if(req.obj_id.data == 16 || req.obj_id.data == 31)
        {
            cout << "--------------------------------------------------" << endl;
            cout << "Object: " << model_names[req.obj_id.data-1] << " will be put inside another 5cm" << endl;
            cout << "--------------------------------------------------" << endl;
            tmp.y += 0.05;
        }
        waypoints_rack.push_back(tmp);


        int idx = 0;
        vector<double> angles(NO_JOINTS);
        vector<vector<double> > trajectory;
        vector<vector<double> > trajectory_rack;


        for(int i=0; i<waypoints.size(); i++)
        {
            success = computeToteIK(waypoints[i], idx, angles, suction_orientation);
//            if(!success)
//                break;
            trajectory.push_back(angles);
        }

        if(success)
        {
            // compute trajectory angles to go inside bin
            for(int i=0; i<waypoints_rack.size(); i++)
            {
                cout << waypoints_rack[i].x << " " << waypoints_rack[i].y << " " << waypoints_rack[i].z << endl;
                success = computeMiddleIK(waypoints_rack[i], req.bin_num.data, angles);
//                if(!success)
//                    break;
                trajectory_rack.push_back(angles);
            }
            cout << "No. of rack way_points: " << waypoints_rack.size() << endl;
        }
        if(success)
        {

            for(int j=0; j<NO_JOINTS; j++)
                res.jts_reach_tote.data.push_back(tote_above_psn[j]);

            for(int i=0; i<trajectory.size(); i++)
            {
                for(int j=0; j<trajectory[i].size(); j++)
                    res.jts_reach_tote.data.push_back(trajectory[i][j]);
            }

            for(int i=trajectory.size()-2; i>=0; i--)
            {
                for(int j=0; j<trajectory[i].size(); j++)
                    res.jts_back_tote.data.push_back(trajectory[i][j]);
            }
            for(int j=0; j<NO_JOINTS; j++)
                res.jts_back_tote.data.push_back(tote_retract_psn[j]);

            for(int i=0; i<trajectory_rack.size(); i++)
            {
                for(int j=0; j<trajectory_rack[i].size(); j++)
                    res.jts_reach_rack.data.push_back(trajectory_rack[i][j]);
            }

//            res.flag_grip.data = false;
            return true;
            cout << "----------------------------------------" << endl << endl;
            cout << "Object name: " << model_names[req.obj_id.data-1] << endl;
            cout << "Request to put into bin " << req.bin_num.data << endl << endl;
            cout << "----------------------------------------" << endl;
        }
        else
            return false;
    }

}

bool contiTrajCallback(trajectory_rpt::TrajStow::Request &req, trajectory_rpt::TrajStow::Response &res)
{
    vector<geometry_msgs::Point> waypoints;

    geometry_msgs::Point initial;
    initial.x = req.bin_centroid.data[0];
    initial.y = req.bin_centroid.data[1]-0.2;
    initial.z = req.bin_centroid.data[2];


    geometry_msgs::Point p1;
    p1.x = req.bin_centroid.data[0];
    p1.y = req.bin_centroid.data[1]+0.1;
    p1.z = req.bin_centroid.data[2];


    geometry_msgs::Point p2;
    p2.x = req.bin_centroid.data[0];
    p2.y = req.bin_centroid.data[1]+0.1;
    p2.z = req.bin_centroid.data[2]-0.05;

    waypoints.push_back(initial);
    waypoints.push_back(p1);
    waypoints.push_back(p2);

    trajectory_rpt::ContinuousTrajGen contTraj;
    vector<double> angles(7,0.0);

    for(int i=0; i<waypoints.size(); i++)
    {
        bool success = computeMiddleIK(waypoints[i], req.bin_num.data, angles);
        IkReal eerot[9],eetrans[3] = {0,0,0};
        IkReal *lst_jts = (IkReal*) malloc(sizeof(IkReal)*NO_JOINTS);
        for(int j=0; j<NO_JOINTS; j++)
            lst_jts[j] = angles[j];
        ComputeFk(lst_jts, eetrans, eerot);

        if(i==0)
        {
            for(int j=0; j<NO_JOINTS; j++)
            {
                contTraj.request.initial_jts.data.push_back(angles[j]);
            }
        }
        else if(i==1)
        {
            contTraj.request.point1.x = waypoints[i].x-VACUUM_DIM_X;
            contTraj.request.point1.y = waypoints[i].y-VACUUM_DIM_Y;
            contTraj.request.point1.z = waypoints[i].z-VACUUM_DIM_Z;
            for(int j=0; j<9; j++)
                contTraj.request.rotation_matrix1.data.push_back(eerot[j]);
        }
        else if(i==2)
        {
            contTraj.request.point2.x = waypoints[i].x-VACUUM_DIM_X;
            contTraj.request.point2.y = waypoints[i].y-VACUUM_DIM_Y;
            contTraj.request.point2.z = waypoints[i].z-VACUUM_DIM_Z;
            for(int j=0; j<9; j++)
                contTraj.request.rotation_matrix2.data.push_back(eerot[j]);
        }

//        trajectory.push_back(angles);
    }
    if(continuousTrajService.call(contTraj))
    {
        res.jts_reach_rack.data = contTraj.response.continuous_traj.data;
        return true;
    }
    else
        return false;
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_stow");
    ros::NodeHandle node;

    ros::ServiceServer service_traj_stow;

    service_traj_stow = node.advertiseService("/trajectory/stow", getTrajCallback);

    ros::ServiceServer service_continuous_traj;

    service_continuous_traj = node.advertiseService("/trajectory/continuous", contiTrajCallback);

    continuousTrajService = node.serviceClient<trajectory_rpt::ContinuousTrajGen>("/continuous_traj");

    ROS_INFO("Ready to get trajectory for stow pick task.");

    while(ros::ok())
        ros::spinOnce();
    return 0;

}
