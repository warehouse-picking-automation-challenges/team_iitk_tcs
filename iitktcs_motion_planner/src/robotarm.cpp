#include "iitktcs_motion_planner/robotarm.h"
#include <tf_conversions/tf_eigen.h>

RobotArm::RobotArm()
{
    // arm group stuff
    this->current_group = 0;
    this->arm_group_straight = new moveit::planning_interface::MoveGroup ("manipulator_straight");
    this->arm_group_mid = new moveit::planning_interface::MoveGroup ("manipulator_mid");
    this->arm_group_bend = new moveit::planning_interface::MoveGroup ("manipulator_bend");
    this->arm_group_gripper = new moveit::planning_interface::MoveGroup ("manipulator_gripper");

    this->arm_group_straight->setPlanningTime(6);
    this->arm_group_mid->setPlanningTime(6);
    this->arm_group_bend->setPlanningTime(6);
    this->arm_group_gripper->setPlanningTime(6);

    this->arm_group_straight->setPlannerId("RRTConnectkConfigDefault");
    this->arm_group_mid->setPlannerId("RRTConnectkConfigDefault");
    this->arm_group_bend->setPlannerId("RRTConnectkConfigDefault");
    this->arm_group_gripper->setPlannerId("RRTConnectkConfigDefault");

    this->arm_group_straight->allowReplanning(true);
    this->arm_group_mid->allowReplanning(true);
    this->arm_group_bend->allowReplanning(true);
    this->arm_group_gripper->allowReplanning(true);

    int no_planning_attempts = 5;
    this->arm_group_straight->setNumPlanningAttempts(no_planning_attempts);
    this->arm_group_mid->setNumPlanningAttempts(no_planning_attempts);
    this->arm_group_bend->setNumPlanningAttempts(no_planning_attempts);
    this->arm_group_gripper->setNumPlanningAttempts(no_planning_attempts);

    // publisher for suction gripper control system
    this->pub_planning_position = this->nh.advertise<std_msgs::Int16MultiArray>
            ("/iitktcs/motion_planner/planning/suction_gripper_pos",100);

    // index [0]- gripper position control in rviz. 0- retracted, 1- extented
    // index [1]- suction position control in rviz. 0- straight, 1- mid, 2- bend
    // index [2]- execution of position onto real hardware system. 0- don't execute, 1- execute
    this->gripper_suction_sys_control.data.resize(3);
    this->gripper_closed = false;

    // marker array for displaying trajectory points
    this->pub_markerarray_traj_points = this->nh.advertise<visualization_msgs::MarkerArray>("/iitktcs_motion_planner/trajectory/points",10);
    this->pub_markerarray_centroid = this->nh.advertise<visualization_msgs::MarkerArray>("/iitktcs_motion_planner/object/centroid",10);
    this->pub_markerarray_axis = this->nh.advertise<visualization_msgs::MarkerArray>("/iitktcs_motion_planner/object/axis",10);
    this->pub_markerarray_normal = this->nh.advertise<visualization_msgs::MarkerArray>("/iitktcs_motion_planner/object/normal",10);

    // adding collision objects and collision plates for rack
    this->planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    this->planning_scene_diff_publisher = this->nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    this->plan_result = new moveit::planning_interface::MoveItErrorCode ();
    this->min_path_value = 1000000; // set the value for min path distance to a large value

    this->coll = new moveit_msgs::CollisionObject;
    this->coll1 = new moveit_msgs::CollisionObject;
    this->coll2 = new moveit_msgs::CollisionObject;
    this->coll3 = new moveit_msgs::CollisionObject;
    this->coll4 = new moveit_msgs::CollisionObject;
    this->coll5 = new moveit_msgs::CollisionObject;
    this->display_publisher = this->nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    std::vector<double> tmp_corner_a, tmp_corner_b;
    vector<double> suction_mid_min_max;
    while(tmp_corner_a.size() == 0 || tmp_corner_b.size() == 0)
    {
        this->nh.getParam("corners_bin_a", tmp_corner_a);
        this->nh.getParam("corners_bin_b", tmp_corner_b);
        this->nh.getParam("cartesian_path_fraction", this->cartesian_path_fraction);
        this->nh.getParam("no_bins", this->no_bins);
        this->nh.getParam("retrieve_pick_height", this->retrieve_pick_height);
        this->nh.getParam("distance_along_normal", this->distance_along_normal);
        this->nh.getParam("approach_vector", this->approach_vector);
        this->nh.getParam("suction_mid_min_max", suction_mid_min_max);
        cout << "Waiting to get parameter" << endl;
    }
//    cout << "suction_mid_min_max: " << suction_mid_min_max[0] << " " << suction_mid_min_max[1] << endl;
    cout << "Retrieve height: " << this->retrieve_pick_height << endl;
    for(int i=0; i<4; i++)
    {
        tf::Vector3 v(tmp_corner_a[i*3], tmp_corner_a[i*3+1], tmp_corner_a[i*3+2]);
        this->bin_corners.push_back(v);
    }
    for(int i=0; i<4; i++)
    {
        tf::Vector3 v(tmp_corner_b[i*3], tmp_corner_b[i*3+1], tmp_corner_b[i*3+2]);
        this->bin_corners.push_back(v);
    }

    // find bin normal, axis vectors and centroids
    tf::Vector3 vertical, horizontal;
    for(int i=0; i<this->no_bins; i++)
    {
        double sum_x=0.0, sum_y=0.0, sum_z=0.0;
        for(int j=0; j<4; j++)
        {
            sum_x += this->bin_corners[4*i+j].getX();
            sum_y += this->bin_corners[4*i+j].getY();
            sum_z += this->bin_corners[4*i+j].getZ();
        }
        // take average of all points to find the bin centroid
        tf::Vector3 centroid(sum_x/4, sum_y/4, sum_z/4);
        this->bin_centroid.push_back(centroid);

        // top minus bottom points for vertical vector
        double vx = this->bin_corners[4*i].getX()-this->bin_corners[4*i+2].getX();
        double vy = this->bin_corners[4*i].getY()-this->bin_corners[4*i+2].getY();
        double vz = this->bin_corners[4*i].getZ()-this->bin_corners[4*i+2].getZ();

        tf::Vector3 vv(vx, vy, vz);
        vv.normalize();
        vertical += vv;

        // right minus left points for horizontal vector
        double hx = this->bin_corners[4*i+1].getX()-this->bin_corners[4*i].getX();
        double hy = this->bin_corners[4*i+1].getY()-this->bin_corners[4*i].getY();
        double hz = this->bin_corners[4*i+1].getZ()-this->bin_corners[4*i].getZ();

        tf::Vector3 hv(hx, hy, hz);
        hv.normalize();
        horizontal += hv;
    }

    // bin normal and axis
    vertical.normalize();   horizontal.normalize();
    this->bin_normal = horizontal.cross(vertical);
    this->bin_axis = horizontal;
    this->bin_top_vector = this->bin_normal.cross(this->bin_axis);

//    cout << this->bin_axis.getX() << " " << this->bin_axis.getY() << " " << this->bin_axis.getZ() << endl;
//    cout << this->bin_normal.getX() << " " << this->bin_normal.getY() << " " << this->bin_normal.getZ() << endl;
//    cout << this->bin_centroid[0].getX() << " " << this->bin_centroid[0].getY() << " " << this->bin_centroid[0].getZ() << endl;

    vector< vector<string> > box_plate_names(2, vector<string> (5,""));
    box_plate_names[0][0] = "zplate1";
    box_plate_names[0][1] = "zplate2";
    box_plate_names[0][2] = "zplate3";
    box_plate_names[0][3] = "zplate4";
    box_plate_names[0][4] = "zplate9";
    box_plate_names[1][0] = "zplate5";
    box_plate_names[1][1] = "zplate6";
    box_plate_names[1][2] = "zplate7";
    box_plate_names[1][3] = "zplate8";
    box_plate_names[1][4] = "zplate10";

    for(int i=0; i<2; i++)
        addPlates(i, box_plate_names[i]);

    // specify the touch links that could touch the attached object
    this->touch_links.push_back("bellow_link");
    this->touch_links.push_back("suction_base_link");
    this->touch_links.push_back("bottom_motor_link");
    this->touch_links.push_back("gripper_body_link");
    this->touch_links.push_back("left_finger_link");
    this->touch_links.push_back("right_finger_link");
    this->is_attached_object_present = false;
    this->added_object = false;

    this->client_clear_octomap = nh.serviceClient<iitktcs_msgs_srvs::static_point_cloud>("/iitktcs/utils/publishoctomap");
    this->client_checkclear_protective_stop = nh.serviceClient<iitktcs_msgs_srvs::CheckClearProtectiveStop>
            ("iitktcs/motion_planner/check_clear_protective_stop");
    this->client_pub_octomap = nh.serviceClient<iitktcs_msgs_srvs::static_point_cloud>("/iitktcs/utils/octomap");
    this->client_vacuum_cleaner_control = nh.serviceClient<iitktcs_msgs_srvs::gripper_suction_controller>("/iitktcs/gripper_suction_controller");

//    const std::string object_names[]=
//    {
//        "toilet_brush","avery_binder","balloons","band_aid_tape","bath_sponge","black_fashion_gloves","burts_bees_baby_wipes","colgate_toothbrush_4pk","composition_book","crayons"
//        ,"duct_tape","epsom_salts","expo_eraser","fiskars_scissors","flashlight","glue_sticks","hand_weight","hanes_socks" ,"hinged_ruled_index_cards","ice_cube_tray","irish_spring_soap"
//        ,"laugh_out_loud_jokes","marbles","measuring_spoons","mesh_cup","mouse_traps","pie_plates","plastic_wine_glass","poland_spring_water","reynolds_wrap","robots_dvd"
//        ,"robots_everywhere","scotch_sponges","speed_stick","white_facecloth","table_cloth","tennis_ball_container","ticonderoga_pencils","tissue_box","windex"
//    };
//    this->no_objects = 40;
//    for(int i=0; i<no_objects; i++)
//        this->model_names.push_back(object_names[i]);

    vector<string> obj_names;
    vector<int> black;

    cout << "waiting for object names" << endl;
    while(obj_names.size() == 0)
    {
        nh.getParam("/ARC17_OBJECT_NAMES/",obj_names);
    }

    bool got_black = nh.getParam("/black_object/",black);
    for(int i=0; i<black.size(); i++)
    {
        this->non_planning_item.push_back((int) black[i]);// mesh cup
        cout << "Black: " << this->non_planning_item[i] << endl;
    }

    this->no_objects = obj_names.size();
    for(int i=0; i<no_objects; i++)
    {
        this->model_names.push_back(obj_names[i]);
        cout << i+1 << " " << this->model_names.back() << endl;
    }

    this->is_drop_plan = false;
    this->is_retrieval_plan = false;
    this->mesh_cup_gripper = false;

    this->publisher_vacuum_control = nh.advertise<std_msgs::Int16>("/iitktcs/arduino/vacuum_ctrl", 100);
    this->subscriber_flow_sensor = nh.subscribe<rosserial_arduino::Adc>
            ("/iitktcs/arduino/flow_Sensor", 100, &RobotArm::flowSensorCallback, this);
    this->flag_get_flow_sensor_reading = false;
}

RobotArm::~RobotArm(){}

void RobotArm::flowSensorCallback(const rosserial_arduino::AdcConstPtr &adc_ptr)
{
    if(this->flag_get_flow_sensor_reading)
    {
        this->flow_sensor_adc = (double) adc_ptr->adc0;
        this->flag_get_flow_sensor_reading = false;
    }
}

// Function to get the current joints of the robots
vector<double> RobotArm::getCurrentJoints()
{
    std::vector<double> group_variable_values;

    this->arm_group_straight->getCurrentState()->copyJointGroupPositions(this->arm_group_straight->getCurrentState()->getRobotModel()->getJointModelGroup(this->arm_group_straight->getName()), group_variable_values);
    return group_variable_values;
}

bool RobotArm::checkJointsStateReached(vector<double> &desired_jts)
{
    bool goal_achieved = false;

    double time = ros::Time::now().toSec();
    double duration;

    int start_jt = 0;
    while(!goal_achieved)
    {
        vector<double> current_jts = this->getCurrentJoints();
        double avg_pos_diff = 0;

        for(int i = start_jt; i < desired_jts.size(); i++)
        {
            avg_pos_diff += fabs(current_jts[i] - desired_jts[i]);

        }

        avg_pos_diff /= desired_jts.size();

        // wait until avg joint position diff is approx. 1 degrees
        if(avg_pos_diff < 0.02)// avg_pos_diff is in radians hence 0.02 rad =0.02*57.1 =1.142 degrees
            goal_achieved = true;

        duration = ros::Time::now().toSec() - time;
        if(duration > 5) // if in loop for more than 5 secs then break out
        {
            ROS_INFO("Time limit exceeded to reach desired position");
            if(avg_pos_diff < 0.05)// if avg is less than 2.855 degrees then consider goal achieved
                goal_achieved = true;
            else
                goal_achieved = false;
            break;
        }
    }

    if(goal_achieved)
        return true;
    else
        return false;
}

// Function for finding motion plan for gripper using cartesian waypoints.
// Two waypoints are generated along the normal of the object.
bool RobotArm::sendCartesianGripperPoseGoal(geometry_msgs::Point &trg_centroid,
                                            tf::Matrix3x3 &trg_orientation)
{
    Eigen::Affine3d trg_pose;
    for(int r=0; r<3; r++)
    {
        for(int c=0; c<3; c++)
        {
            trg_pose(r,c) = trg_orientation[r][c];
        }
    }
    trg_pose(0,3) = trg_centroid.x;
    trg_pose(1,3) = trg_centroid.y;
    trg_pose(2,3) = trg_centroid.z;
    trg_pose(3,0) = trg_pose(3,1) = trg_pose(3,2) = 0.0;
    trg_pose(3,3) = 1.0;
    cout << "target pose:\n"
         << trg_pose(0,0) << " " << trg_pose(0,1) << " " << trg_pose(0,2) << " " << trg_pose(0,3) << " " << endl
         << trg_pose(1,0) << " " << trg_pose(1,1) << " " << trg_pose(1,2) << " " << trg_pose(1,3) << " " << endl
         << trg_pose(2,0) << " " << trg_pose(2,1) << " " << trg_pose(2,2) << " " << trg_pose(2,3) << " " << endl
         << trg_pose(3,0) << " " << trg_pose(3,1) << " " << trg_pose(3,2) << " " << trg_pose(3,3) << " " << endl;

    tf::Vector3 normal(trg_pose(0,0), trg_pose(1,0), trg_pose(2,0));
    tf::Vector3 centroid(trg_centroid.x, trg_centroid.y, trg_centroid.z);
    tf::Vector3 pt = this->getEndpt(normal, centroid, -0.05); // at distance of 5cm in opposite direction of normal

    std::vector<geometry_msgs::Pose> waypoints;

    trg_pose(0,3) = pt.getX();
    trg_pose(1,3) = pt.getY();
    trg_pose(2,3) = pt.getZ();

    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(trg_pose, pose);

    waypoints.push_back(pose);

    pose.position.x = trg_centroid.x;
    pose.position.y = trg_centroid.y;
    pose.position.z = trg_centroid.z;

    waypoints.push_back(pose);

    cout << "Attempt with the gripper" << endl;

    if(this->genWayPointsPlan(waypoints))
        return true;
    else
        return false;
}

// Takes input target pose. Computes waypoints for path generation
// Based on normal vector 2 waypoints are generated. One at tip of normal at 3cm frm centroid and 2nd is centroid itself
bool RobotArm::genCartesianPlanPose(geometry_msgs::Point &trg_centroid, tf::Matrix3x3 &trg_orientation)
{
    Eigen::Affine3d trg_pose_matrix;
    for(int r=0; r<3; r++)
    {
        for(int c=0; c<3; c++)
        {
            trg_pose_matrix(r,c) = trg_orientation[r][c];
        }
    }
    trg_pose_matrix(0,3) = trg_centroid.x; trg_pose_matrix(1,3) = trg_centroid.y; trg_pose_matrix(2,3) = trg_centroid.z;
    trg_pose_matrix(3,0) = trg_pose_matrix(3,1) = trg_pose_matrix(3,2) = 0.0; trg_pose_matrix(3,3) = 1.0;
    trg_pose_matrix(0,3) = 0.0; trg_pose_matrix(1,3) = 0.0; trg_pose_matrix(2,3) = 0.0;

    tf::Vector3 axis, normal;
    tf::Vector3 centroid(trg_centroid.x, trg_centroid.y, trg_centroid.z);
//    tf::Vector3 approach_vector(this->approach_vector[0],this->approach_vector[1],this->approach_vector[2]);
    tf::Vector3 approach_vector(0,0,1);
    // store pose interms of waypoints for both the provided axis and with inverted axis.
    // here we assume that the normal is always opposite to the approach direction and protruding from object surface
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose;

    vector<tf::Vector3> path_array; // vector for plotting waypoints
    tf::Vector3 path_points;

    tf::Vector3 pt1, pt2;

    if(this->current_group == 0 || this->current_group == 1 || this->current_group == 2)
    {
        // x-suction: +ve of object axis
        // y-suction: +ve of object normal
        // see the urdf model
        axis.setX(trg_pose_matrix(0,0)); axis.setY(trg_pose_matrix(1,0)); axis.setZ(trg_pose_matrix(2,0));
        normal.setX(trg_pose_matrix(0,1)); normal.setY(trg_pose_matrix(1,1)); normal.setZ(trg_pose_matrix(2,1));

        plotAxisNormalCentroid(axis, normal, centroid);

        // at distance of 3cm in the direction of normal
        pt1 = this->getEndpt(normal, centroid, this->distance_along_normal);
        // at distance of 10cm in the direction of z-axis from the point pt1
        pt2 = this->getEndpt(approach_vector, pt1, 0.1);
        cout << pt1.getX() << " " << pt1.getY() << " " << pt1.getZ() << endl;
        cout << pt2.getX() << " " << pt2.getY() << " " << pt2.getZ() << endl;
        trg_pose_matrix(0,3) = pt2.getX();
        trg_pose_matrix(1,3) = pt2.getY();
        trg_pose_matrix(2,3) = pt2.getZ();

        tf::poseEigenToMsg(trg_pose_matrix, pose);

        // at distance of 10cm in the direction of z-axis from the point pt1
        path_points.setX(pose.position.x);
        path_points.setY(pose.position.y);
        path_points.setZ(pose.position.z);
        path_array.push_back(path_points);
        waypoints.push_back(pose);
    }
    else if(this->current_group == 3)
    {
        // x-gripper: -ve of object normal
        // y-gripper: +ve of object axis
        // see the urdf model
        normal.setX(trg_pose_matrix(0,0)); normal.setY(trg_pose_matrix(1,0)); normal.setZ(trg_pose_matrix(2,0));
        axis.setX(trg_pose_matrix(0,1)); axis.setY(trg_pose_matrix(1,1)); axis.setZ(trg_pose_matrix(2,1));

        normal *= -1;
        plotAxisNormalCentroid(axis, normal, centroid);

        // at distance of 3cm in the direction of normal
        pt1 = this->getEndpt(normal, centroid, this->distance_along_normal);

#if(USE_EXTRA_POINT_APPROACH_VECTOR_GRIPPER)
        // at distance of 10cm in the direction of z-axis from the point pt1
        pt2 = this->getEndpt(approach_vector, pt1, 0.1);

        trg_pose_matrix(0,3) = pt2.getX();
        trg_pose_matrix(1,3) = pt2.getY();
        trg_pose_matrix(2,3) = pt2.getZ();

        tf::poseEigenToMsg(trg_pose_matrix, pose);

        // at distance of 10cm in the direction of z-axis from the point pt1
        path_points.setX(pose.position.x);
        path_points.setY(pose.position.y);
        path_points.setZ(pose.position.z);
        path_array.push_back(path_points);
        waypoints.push_back(pose);
#endif
    }

    // point at 3cm along the normal to the object centroid
    pose.position.x = pt1.getX();
    pose.position.y = pt1.getY();
    pose.position.z = pt1.getZ();
    path_points.setX(pose.position.x);
    path_points.setY(pose.position.y);
    path_points.setZ(pose.position.z);
    path_array.push_back(path_points);
    waypoints.push_back(pose);

    // point at the object centroid
    pose.position.x = trg_centroid.x;
    pose.position.y = trg_centroid.y;
    pose.position.z = trg_centroid.z;
    path_points.setX(pose.position.x);
    path_points.setY(pose.position.y);
    path_points.setZ(pose.position.z);
    path_array.push_back(path_points);
    waypoints.push_back(pose);

    sphereMarker(path_array, this->pub_markerarray_centroid);

    if(this->genWayPointsPlan(waypoints))
        return true;
    else
        return false;
}

// generation of motion plan given a set of waypoints
bool RobotArm::genWayPointsPlan(std::vector<geometry_msgs::Pose> &waypoints)
{
    // We want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively
    // disabling it.
    //    double time = ros::Time::now().toSec();

    double velocity_profile = 0.1;
    // change velocity profile only in case of object present or retrieval
    if(this->is_retrieval_plan || this->is_drop_plan)
    {
        const string obj_weight_str = "/" + this->model_names[object_id-1] + "/weight";
        int obj_weight = 100;
        double time = ros::Time::now().toSec();
//        string obj_weight;
//        int string_length = 0;
//        while(string_length==100)
        while(obj_weight == 100)
        {
            this->nh.getParam(obj_weight_str, obj_weight);
            cout << "Waiting for parameters: " << this->model_names[object_id-1].c_str() << endl;
            double time2 = ros::Time::now().toSec();
            if((time2-time) > 2) // if time waited for parameter is more than 2 seconds then break
            {
                cout << "Could not get weight parameter" << endl;
                break;
            }
//            string_length = obj_weight.size();
        }

//        if(strcmp(obj_weight.c_str(), "medium") == 0)
        if(obj_weight == 0)
        {
            cout << "Retrieve Medium object: 0.25 scale" << endl;
            velocity_profile = 0.08;
        }
//        else if(strcmp(obj_weight.c_str(), "heavy") == 0)
        else if(obj_weight == 1)
        {
            cout << "Retrieve Heavy object: 0.025 scale" << endl;
            velocity_profile = 0.025;
        }
        else
        {
            cout << "Default velocity: 0.025 scale" << endl;
            velocity_profile = 0.025;
        }
    }
    else
        cout << "Max velocity profile: " << velocity_profile << endl;

    moveit_msgs::RobotTrajectory trajectory;
    double fraction;
    {
        if(this->current_group == 0)
        {
            cout << "Finding cartesian plan with suction straight" << endl;
            for(int i=0; i<waypoints.size(); i++)
            {
                geometry_msgs::Pose pose = waypoints[i];
                cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;
            }
            this->arm_group_straight->setMaxVelocityScalingFactor(velocity_profile);
            fraction = this->arm_group_straight->computeCartesianPath(waypoints, 0.01, 0.0, trajectory,
                                                                      true, this->plan_result);
        }
        else if(this->current_group == 1)
        {
            this->arm_group_mid->setMaxVelocityScalingFactor(velocity_profile);
            fraction = this->arm_group_mid->computeCartesianPath(waypoints, 0.01, 0.0, trajectory,
                                                                 true, this->plan_result);
        }
        else if(this->current_group == 2)
        {
            this->arm_group_bend->setMaxVelocityScalingFactor(velocity_profile);
            fraction = this->arm_group_bend->computeCartesianPath(waypoints, 0.01, 0.0, trajectory,
                                                                  true, this->plan_result);
        }
        else if(this->current_group == 3)
        {
            this->arm_group_gripper->setMaxVelocityScalingFactor(velocity_profile);
            fraction = this->arm_group_gripper->computeCartesianPath(waypoints, 0.01, 0.0, trajectory,
                                                                     true, this->plan_result);
        }
        else
        {
            cout << "Error in choosing arm group" << endl;
            return false;
        }

        cout << "**** " << fraction*100 << "% path plan found **** " << endl;
        if(fraction < this->cartesian_path_fraction)
        {
            return false;
        }
    }

    //    cout << "Time for finding trajectory :" << ros::Time::now().toSec()-time << endl;

    if(this->current_group == 0)
    {
        robot_trajectory::RobotTrajectory rt(this->arm_group_straight->getCurrentState()->getRobotModel(),
                                             "manipulator_straight");
        // The trajectory needs to be modified so it will include velocities as well.psn_goal
        // First to create a RobotTrajectory object
        rt = robot_trajectory::RobotTrajectory (this->arm_group_straight->getCurrentState()->getRobotModel(),
                                                "manipulator_straight");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*this->arm_group_straight->getCurrentState(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt, velocity_profile);

        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        // Finally plan and execute the trajectory
        this->my_plan_dummy.trajectory_ = trajectory;

        return true;
    }
    else if(this->current_group == 1)
    {
        robot_trajectory::RobotTrajectory rt(this->arm_group_mid->getCurrentState()->getRobotModel(),
                                             "manipulator_mid");
        // The trajectory needs to be modified so it will include velocities as well.psn_goal
        // First to create a RobotTrajectory object
        rt = robot_trajectory::RobotTrajectory(this->arm_group_mid->getCurrentState()->getRobotModel(),
                                               "manipulator_mid");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*this->arm_group_mid->getCurrentState(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        // Fourth compute computeTimeStamps

        bool success = iptp.computeTimeStamps(rt, velocity_profile);

        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        // Finally plan and execute the trajectory
        this->my_plan_dummy.trajectory_ = trajectory;

        return true;
    }
    else if(this->current_group == 2)
    {
        robot_trajectory::RobotTrajectory rt(this->arm_group_bend->getCurrentState()->getRobotModel(),
                                             "manipulator_bend");
        // The trajectory needs to be modified so it will include velocities as well.psn_goal
        // First to create a RobotTrajectory object
        rt = robot_trajectory::RobotTrajectory(this->arm_group_bend->getCurrentState()->getRobotModel(),
                                               "manipulator_bend");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*this->arm_group_bend->getCurrentState(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt, velocity_profile);

        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        // Finally plan and execute the trajectory
        this->my_plan_dummy.trajectory_ = trajectory;

        return true;
    }
    else if(this->current_group == 3)
    {
        robot_trajectory::RobotTrajectory rt(this->arm_group_gripper->getCurrentState()->getRobotModel(),
                                             "manipulator_gripper");
        // The trajectory needs to be modified so it will include velocities as well.psn_goal
        // First to create a RobotTrajectory object
        rt = robot_trajectory::RobotTrajectory(this->arm_group_gripper->getCurrentState()->getRobotModel(),
                                               "manipulator_gripper");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*this->arm_group_gripper->getCurrentState(), trajectory);

        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt, velocity_profile);

        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);

        // Finally plan and execute the trajectory
        this->my_plan_dummy.trajectory_ = trajectory;

        return true;
    }
    else
    {
        cout << "Error in choosing arm group" << endl;
        return false;
    }
}

void RobotArm::displayTrajectoryPoints(moveit::planning_interface::MoveGroup::Plan &plan, string type)
{
    cout << "Reached display trajectory " << endl;
    int step_size;
    if(strcmp(type.c_str(),"joint")==0)
        step_size = 1;
    else if(strcmp(type.c_str(),"cartesian")==0)
        step_size = 4;

    vector<double> joint_positions(NO_JOINTS);
    vector<tf::Vector3> points;

    Eigen::Affine3d eeMatrix = Eigen::Affine3d::Identity();

    for(int i=0; i<plan.trajectory_.joint_trajectory.points.size(); i+=step_size)
    {
        for(int j=0; j<plan.trajectory_.joint_trajectory.points[i].positions.size(); j++)
        {
            joint_positions[j] = plan.trajectory_.joint_trajectory.points[i].positions[j];
        }
        eeMatrix = this->getFK(joint_positions,false);
        tf::Vector3 v(eeMatrix(0,3), eeMatrix(1,3), eeMatrix(2,3));
        points.push_back(v);
    }

    sphereMarker(points, this->pub_markerarray_traj_points);

    return;
}

// function to get forward kinematics of the robot
// provides forward kinematics for both the current state of robot and for a specified joints
Eigen::Affine3d RobotArm::getFK(vector<double> joint_values, bool fk_current)
{
    if(fk_current)
    {
        if(this->current_group == 0)
        {
            this->kinematic_state = this->arm_group_straight->getCurrentState();
            this->joint_model_group = this->kinematic_state->getRobotModel()->getJointModelGroup(this->arm_group_straight->getName());
        }
        else if(this->current_group == 1)
        {
            this->kinematic_state = this->arm_group_mid->getCurrentState();
            this->joint_model_group = this->kinematic_state->getRobotModel()->getJointModelGroup(this->arm_group_mid->getName());
        }
        else if(this->current_group == 2)
        {
            this->kinematic_state = this->arm_group_bend->getCurrentState();
            this->joint_model_group = this->kinematic_state->getRobotModel()->getJointModelGroup(this->arm_group_bend->getName());
        }
        else if(this->current_group == 3)
        {
            this->kinematic_state = this->arm_group_gripper->getCurrentState();
            this->joint_model_group = this->kinematic_state->getRobotModel()->getJointModelGroup(this->arm_group_gripper->getName());
        }
        this->kinematic_state->copyJointGroupPositions(this->joint_model_group, joint_values);

        for(std::size_t i = 0; i < joint_values.size(); ++i)
            cout << joint_values[i] << " ";
        cout << endl;
    }
    else
    {
        if(this->current_group == 0)
            this->kinematic_state = this->arm_group_straight->getCurrentState();
        else if(this->current_group == 1)
            this->kinematic_state = this->arm_group_mid->getCurrentState();
        else if(this->current_group == 2)
            this->kinematic_state = this->arm_group_bend->getCurrentState();
        else if(this->current_group == 3)
            this->kinematic_state = this->arm_group_gripper->getCurrentState();

        vector<string> joint_names;
        joint_names.push_back("shoulder_pan_joint");
        joint_names.push_back("shoulder_lift_joint");
        joint_names.push_back("elbow_joint");
        joint_names.push_back("wrist_1_joint");
        joint_names.push_back("wrist_2_joint");
        joint_names.push_back("wrist_3_joint");

        this->kinematic_state->setVariablePositions(joint_names, joint_values);
    }

    Eigen::Affine3d end_effector_state;
    if(this->current_group == 0)
    {
//        cout << "Computing FK for link: " << this->arm_group_straight->getEndEffectorLink() << endl;
        end_effector_state = this->kinematic_state->getGlobalLinkTransform(this->arm_group_straight->getEndEffectorLink());
//        end_effector_state = this->kinematic_state->getGlobalLinkTransform("nozele");
    }
    else if(this->current_group == 1)
    {
//        cout << "Computing FK for link: " << this->arm_group_mid->getEndEffectorLink() << endl;
        end_effector_state = this->kinematic_state->getGlobalLinkTransform(this->arm_group_mid->getEndEffectorLink());
    }
    else if(this->current_group == 2)
    {
//        cout << "Computing FK for link: " << this->arm_group_bend->getEndEffectorLink() << endl;
        end_effector_state = this->kinematic_state->getGlobalLinkTransform(this->arm_group_bend->getEndEffectorLink());
    }
    else if(this->current_group == 3)
    {
//        cout << "Computing FK for link: " << this->arm_group_bend->getEndEffectorLink() << endl;
        end_effector_state = this->kinematic_state->getGlobalLinkTransform(this->arm_group_gripper->getEndEffectorLink());
    }

    tf::Vector3 axis, normal;
    tf::Vector3 centroid(end_effector_state(0,3), end_effector_state(1,3), end_effector_state(2,3));
    if(this->current_group == 0 || this->current_group == 1 || this->current_group == 2)
    {
        axis.setX(end_effector_state(0,0)); axis.setY(end_effector_state(1,0)); axis.setZ(end_effector_state(2,0));
        normal.setX(end_effector_state(0,1)); normal.setY(end_effector_state(1,1)); normal.setZ(end_effector_state(2,1));
    }
    else if(this->current_group == 3)
    {
        axis.setX(end_effector_state(0,1)); axis.setY(end_effector_state(1,1)); axis.setZ(end_effector_state(2,1));
        normal.setX(end_effector_state(0,0)); normal.setY(end_effector_state(1,0)); normal.setZ(end_effector_state(2,0));
        normal *= -1;
    }

    plotAxisNormalCentroid(axis, normal, centroid);
    return end_effector_state;
}

// Function to get the end point a vector at distance from the start point
// here start point is the base of the vector and end point is tip(arrow) of the vector
// keep distance positive if you want end pt along the vector
// keep distance negative if you want end pt opposite to the vector
tf::Vector3 RobotArm::getEndpt(tf::Vector3 vector, tf::Vector3 &start_pt, double distance)
{
    vector.normalize();
    vector *= distance;
    tf::Vector3 end_pt(vector);
    end_pt += start_pt;

    return end_pt;
}

void RobotArm::displayPlannerError(int &error_code)
{
    //    cout << "Error code: " << error_code << endl;
    switch(error_code)
    {
    case 1:
        cout << "Planner: SUCCESS" << endl;
        break;
    case 99999:
        cout << "Planner: FAILURE" << endl;
        break;
    case -1:
        cout << "Planner: PLANNING_FAILED" << endl;
        break;
    case -2:
        cout << "Planner: INVALID_MOTION_PLAN" << endl;
        break;
    case -3:
        cout << "Planner: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE" << endl;
        break;
    case -4:
        cout << "Planner: CONTROL_FAILED" << endl;
        break;
    case -5:
        cout << "Planner: UNABLE_TO_AQUIRE_SENSOR_DATA" << endl;
        break;
    case -6:
        cout << "Planner: TIMED_OUT" << endl;
        break;
    case -7:
        cout << "Planner: PREEMPTED" << endl;
        break;
    case -10:
        cout << "Planner: START_STATE_IN_COLLISION" << endl;
        break;
    case -11:
        cout << "Planner: START_STATE_VIOLATES_PATH_CONSTRAINTS" << endl;
        break;
    case -12:
        cout << "Planner: GOAL_IN_COLLISION" << endl;
        break;
    case -13:
        cout << "Planner: GOAL_VIOLATES_PATH_CONSTRAINTS" << endl;
        break;
    case -14:
        cout << "Planner: GOAL_CONSTRAINTS_VIOLATED" << endl;
        break;
    case -15:
        cout << "Planner: INVALID_GROUP_NAME" << endl;
        break;
    case -16:
        cout << "Planner: INVALID_GOAL_CONSTRAINTS" << endl;
        break;
    case -17:
        cout << "Planner: INVALID_ROBOT_STATE" << endl;
        break;
    case -18:
        cout << "Planner: INVALID_LINK_NAME" << endl;
        break;
    case -19:
        cout << "Planner: INVALID_OBJECT_NAME" << endl;
        break;
    case -21:
        cout << "Planner: FRAME_TRANSFORM_FAILURE" << endl;
        break;
    case -22:
        cout << "Planner: COLLISION_CHECKING_UNAVAILABLE" << endl;
        break;
    case -23:
        cout << "Planner: ROBOT_STATE_STALE" << endl;
        break;
    case -24:
        cout << "Planner: SENSOR_INFO_STALE" << endl;
        break;
    case -31:
        cout << "Planner: NO_IK_SOLUTION" << endl;
        break;
    default:
        cout << "Error type not found" << endl;
        break;
    }
}

bool RobotArm::jointTargetGoal(vector<double> &goal)
{
//    cout << "Reached joint target goal planner" << endl;
    cout << "Reached retrieve after protective stop" << endl;
    double velocity_profile = 0.07;
    this->arm_group_straight->setMaxVelocityScalingFactor(velocity_profile);
    this->is_suction_plan = false;
    for(int i=0; i<4; i++)
    {
        this->current_group = 0;

        int execution_status;
        this->arm_group_straight->setJointValueTarget(goal);

        *(this->plan_result) = this->arm_group_straight->plan(this->my_plan_pose_joint_goal);

        if(this->plan_result->val == 1)
        {
            displayTrajectoryPoints(this->my_plan_pose_joint_goal, "joint");
            if(executePlan(execution_status, this->my_plan_pose_joint_goal))
            {
                cout << "Executed retrieve after protective stop" << endl;
                return true;
            }
            else
            {
                cout << "Failed to execute iteration: " << i+1 << endl;
            }
        }
        else
        {
            cout << "Failed to find plan iteration: " << i+1 << endl;

            iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
            if(this->client_checkclear_protective_stop.call(check_clear_ps))
            {
                if(check_clear_ps.response.flag_protective_stopped.data)
                {
                    cout << "Protective retrieve: Robot was protective stopped and cleared" << endl;
                }
                else
                {
                    cout << "Protective retrieve: Robot is OK!!" << endl;
                }
            }
            else
                cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
        }
    }

    double default_upward_jts[][NO_JOINTS] = {{0.0890376, -1.55013, 1.77058, -1.82832, -1.59681, 0.0956652},
                                              {-0.535401, -1.56516, 1.78588, -1.82982, -1.57258, -0.528362}
                                             };

    double error[2] = {0.0, 0.0};
    int bins = 2;

    for(int i=0; i<bins; i++)
    {
        for(int j=0; j<NO_JOINTS; j++)
        {
            error[i] += (default_upward_jts[i][j] - goal[j])*(default_upward_jts[i][j] - goal[j]);
        }
    }
    int idx = 0;
    if(error[0] < error[1])
    {
        idx = 0;
        cout << "Retry with default jts for bin (0): A" << endl;
    }
    else
    {
        idx = 1;
        cout << "Retry with default jts for bin (1): B" << endl;
    }
    // plan and execute default retrieve plan
    {
        vector<double> goal2(NO_JOINTS);
        for(int j=0; j<NO_JOINTS; j++)
                goal2[j] = default_upward_jts[idx][j];

        int execution_status;
        this->arm_group_straight->setJointValueTarget(goal2);

        *(this->plan_result) = this->arm_group_straight->plan(this->my_plan_pose_joint_goal);

        if(this->plan_result->val == 1)
        {
            displayTrajectoryPoints(this->my_plan_pose_joint_goal, "joint");
            if(executePlan(execution_status, this->my_plan_pose_joint_goal))
            {
                cout << "Executed retrieve after protective stop" << endl;
                return true;
            }
            else
            {
                cout << "Failed to execute in retry plan " << endl;
            }
        }
        else
        {
            cout << "Failed to find retry plan" << endl;

            iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
            if(this->client_checkclear_protective_stop.call(check_clear_ps))
            {
                if(check_clear_ps.response.flag_protective_stopped.data)
                {
                    cout << "Protective retrieve: Robot was protective stopped and cleared" << endl;
                }
                else
                {
                    cout << "Protective retrieve: Robot is OK!!" << endl;
                }
            }
            else
                cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
        }
    }



    return false;
}

// service callback function for fixed joint space target position
bool RobotArm::serviceSingleGoalCallback(iitktcs_msgs_srvs::UR5Goal::Request &req,
                                         iitktcs_msgs_srvs::UR5Goal::Response &res)
{
    cout << "Reached service to send single goal " << endl;
    vector<double> goal;
    this->is_suction_plan = false;
    for(vector<double>::const_iterator it = req.goal.data.begin(); it!=req.goal.data.end(); ++it)
    {
//        cout << *it << " ";
        goal.push_back(*it);
    }
    cout << endl;

    double velocity_profile = 0.3;
    if(req.object_present.data)
    {
        const string obj_weight_str = "/" + this->model_names[object_id-1] + "/weight";
        int obj_weight = 100;
        double time = ros::Time::now().toSec();
        while(obj_weight==100)
        {
            this->nh.getParam(obj_weight_str, obj_weight);
            cout << "Waiting for parameters: " << this->model_names[object_id-1].c_str() << endl;
            double time2 = ros::Time::now().toSec();
            if((time2-time) > 2) // if time waited for parameter is more than 2 seconds then break
            {
                cout << "Could not get weight parameter" << endl;
                break;
            }
        }

        if(obj_weight==0)
        {
            cout << "Drop Medium object: 0.2 scale" << endl;
            velocity_profile = 0.2;
        }
        else if(obj_weight==1)
        {
            cout << "Drop Heavy object: 0.04 scale" << endl;
            velocity_profile = 0.05;
        }
        else
        {
            cout << "Default velocity: 0.04 scale" << endl;
            velocity_profile = 0.05;
        }
    }
    this->arm_group_straight->setMaxVelocityScalingFactor(velocity_profile);

    for(int i=0; i<4; i++)
    {
        this->current_group = 0;

        int execution_status;
        this->arm_group_straight->setJointValueTarget(goal);

        *(this->plan_result) = this->arm_group_straight->plan(this->my_plan_pose_joint_goal);

        if(this->plan_result->val == 1)
        {
            displayTrajectoryPoints(this->my_plan_pose_joint_goal, "joint");
            if(executePlan(execution_status, this->my_plan_pose_joint_goal))
            {
                if(!this->gripper_closed) // if gripper is not closed then straighten the suction and gripper retracted
                {
                    cout << "Gripper is retracted and suction is straight" << endl;
                    this->gripper_suction_sys_control.data[0] = 0;
                    this->gripper_suction_sys_control.data[1] = 0;
                    this->gripper_suction_sys_control.data[2] = 0;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);

                    this->gripper_suction_sys_control.data[0] = 0;
                    this->gripper_suction_sys_control.data[1] = 0;
                    this->gripper_suction_sys_control.data[2] = 1;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);
                }

//                cout << "Reached target position" << endl;

                // in case of drop jts
                if(req.flag_remove_object.data)
                {
#if(PLAN_GRIPPER_MESH_CUP)
                if(this->mesh_cup_gripper)
                {
                    this->gripper_suction_sys_control.data[0] = 2;
                    this->gripper_suction_sys_control.data[1] = 0;
                    this->gripper_suction_sys_control.data[2] = 0;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);

                    this->gripper_suction_sys_control.data[0] = 2;
                    this->gripper_suction_sys_control.data[1] = 0;
                    this->gripper_suction_sys_control.data[2] = 1;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);
                    cout << "Waiting for mesh cup to drop by closing gripper " << endl;
                    sleep(3);

                    this->gripper_suction_sys_control.data[0] = 0;
                    this->gripper_suction_sys_control.data[1] = 2;
                    this->gripper_suction_sys_control.data[2] = 0;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);

                    this->gripper_suction_sys_control.data[0] = 0;
                    this->gripper_suction_sys_control.data[1] = 2;
                    this->gripper_suction_sys_control.data[2] = 1;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);
                    sleep(5);
                    this->gripper_closed = false;
                    // Make use of next step by saying gripper is not closed even though it is closed
                }
#endif
                    if(this->is_attached_object_present)
                    {
#if(ATTACH_OBJECT && ADD_COLLISION_OBJECT)
                        this->detachCollisionObject();
                        if(this->current_group == 0)
                            this->removeCollisionObject("s_st_eef_link");
                        else if(this->current_group == 1)
                            this->removeCollisionObject("s_mid_eef_link");
                        else if(this->current_group == 2)
                            this->removeCollisionObject("s_bend_eef_link");
                        else if(this->current_group == 3)
                            this->removeCollisionObject("gripper_eef_link");
#endif
                    }
                }
                res.success.data = true;
                return true;
            }
            else
            {
                cout << "Failed to execute" << endl;

                iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                if(this->client_checkclear_protective_stop.call(check_clear_ps))
                {
                    if(check_clear_ps.response.flag_protective_stopped.data)
                    {
                        cout << "Single goal robot was protective stopped and cleared" << endl;
                    }
                }
                else
                    cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
            }
        }
        else
            cout << "Failed to find plan" << endl;
    }
    res.success.data = false;
    return true;
}

// service callback function for gripper as end effector and target specified with pose
bool RobotArm::serviceGripperPoseGoalCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req,
                                              iitktcs_msgs_srvs::UR5PoseGoal::Response &res)
{
    cout << "Reached service to send pose goal for gripper" << endl;
#if(VGS)
    // set the end effector name
    // can be provided with end effector link name as well
    string end_effector_name = "gripper";
    this->arm_group_ptr->setEndEffector(end_effector_name);
#endif

    geometry_msgs::Point centroid;
    centroid.x = req.centroid.x;  centroid.y = req.centroid.y; centroid.z = req.centroid.z;
    cout << "centroid:" << centroid.x << " " << centroid.y << " " << centroid.z << endl;

    tf::Vector3 axis(req.axis.x, req.axis.y, req.axis.z);

    // compare the y-axis for end effector in current pose with the axis of the target position
    // if the angle between them is less than 90 degrees then use the same axis else invert it

    Eigen::Affine3d end_effector = this->getFK();

    tf::Vector3 eef_axis(end_effector(0,1), end_effector(1,1), end_effector(2,1));

    double dot_prod = axis.dot(eef_axis);
    if(dot_prod < 0) //cos(90) = cos(-90) = 0
    {
        cout << "-1*axis pose goal attempted" << endl;
        axis *= -1;
    }
    else
    {
        cout << "1*axis pose goal attempted" << endl;
    }
    axis.normalize();

    //    Check whether normal is inside or outside of the object
    //    If the normal inwards within 90degrees of the x-axis of end effector then retain the normal
    //    else invert the normal

    tf::Vector3 normal(req.normal.x, req.normal.y, req.normal.z);

    tf::Vector3 eef_normal(end_effector(0,0), end_effector(1,0), end_effector(2,0));

    dot_prod = normal.dot(eef_normal);
    cout << "eef normal\n"
         << eef_normal.getX() << " "
         << eef_normal.getY() << " "
         << eef_normal.getZ() << " " << endl;
    cout << "obj normal\n"
         << normal.getX() << " "
         << normal.getY() << " "
         << normal.getZ() << " " << endl;
    cout << "dot prod: " << dot_prod << endl;
    if(dot_prod < 0) //cos(90) = cos(-90) = 0
    {
        cout << "Reversed the normal vector" << endl;
        normal *= -1;
    }
    else
    {
        cout << "1*normal pose goal attempted" << endl;
    }
    normal.normalize();

    tf::Vector3 vec = normal.cross(axis);
    tf::Matrix3x3 rotation_matrix_1(normal.getX(), axis.getX(), vec.getX(),
                                    normal.getY(), axis.getY(), vec.getY(),
                                    normal.getZ(), axis.getZ(), vec.getZ());

    if(this->sendCartesianGripperPoseGoal(req.centroid, rotation_matrix_1))
    {
        cout << "Success: Moved the arm to target positionn" << endl;
        return true;
    }
    else
    {
        cout << "Failed to move to target position" << endl;
        // return here itself false not going further because octomap is not there
        // collision is not detected when kinect is rotated bottom
        return false;
    }
}

//bool RobotArm::serviceGripperGeneralizedPlanCallback(iitktcs_msgs_srvs::GripperPoseGoal::Request &req,
//                                                     iitktcs_msgs_srvs::GripperPoseGoal::Response &res)
//{
//    cout << "Reached service for Gripper pose planning" << endl;

//    vector<tf::Vector3> axis_array, inverted_axis_array, normal_array, centroid_array;
//    cout << "No of handles found: " << req.grasp_poses.size() << endl;
//    int num_grasp_poses;
//    if(req.grasp_poses.size() > 5)
//        num_grasp_poses = 5;
//    else
//        num_grasp_poses = req.grasp_poses.size();

//    tf::StampedTransform transform;
//    transform = this->getTransform("camera_link", "world");

//    for(int i=0; i<num_grasp_poses; i++)
//    {
//        iitktcs_msgs_srvs::grasp_pose grasp_pose = req.grasp_poses[i];
//        tf::Vector3 minor_axis(grasp_pose.axis.x, grasp_pose.axis.y, grasp_pose.axis.z);
//        tf::Vector3 normal(grasp_pose.normal.x, grasp_pose.normal.y, grasp_pose.normal.z);

//        tf::Vector3 axis = minor_axis.cross(normal);
//        tf::Vector3 inverted_axis(axis);
//        inverted_axis *= -1;
//        tf::Vector3 centroid(grasp_pose.centroid.x, grasp_pose.centroid.y, grasp_pose.centroid.z);

//        axis = transformPointVector(transform, axis, true);
//        inverted_axis = transformPointVector(transform, inverted_axis, true);
//        normal = transformPointVector(transform, normal, true);
//        centroid = transformPointVector(transform, centroid);

//        cout << centroid.getX() << " " << centroid.getY() << " " << centroid.getZ() << endl;
//        cout << axis.getX() << " " << axis.getY() << " " << axis.getZ() << endl;
//        cout << normal.getX() << " " << normal.getY() << " " << normal.getZ() << endl;
//        cout << "axis length: " << axis.length() << " normal length: " << normal.length() << endl;
//        if(axis.length() < 0.0005 || normal.length() < 0.0005)
//        {
//            cout << "Axis or normal length approx zero" << endl;
//            continue;
//        }
//        else if(centroid.getX() < 0.000001 && centroid.getY() < 0.000001 && centroid.getZ() < 0.000001)
//        {
//            cout << "centroid at origin" << endl;
//            continue;
//        }
//        else if(normal.getZ() < 0.0)
//        {
//            cout << "normal is inverted" << endl;
//            continue;
//        }
//        else
//        {
//            axis_array.push_back(axis);
//            inverted_axis_array.push_back(inverted_axis);
//            normal_array.push_back(normal);
//            centroid_array.push_back(centroid);
//        }
//    }
//    if(centroid_array.size() == 0)
//    {
//        res.success.data = false;
//        return true;
//    }

//    this->object_id = req.object_id.data;
////    this->object_normal.setX(req.normal.x); this->object_normal.setY(req.normal.y); this->object_normal.setZ(req.normal.z);
////    this->object_centroid.setX(req.centroid.x); this->object_centroid.setY(req.centroid.y); this->object_centroid.setZ(req.centroid.z);
//    this->object_attach_pose_world = req.object_pose;

//#if(ADD_COLLISION_OBJECT)
////    this->addPrimitiveCollisionObject(this->object_attach_pose_world, this->object_id, "world");
//    this->addCollisionObject(this->object_attach_pose_world, this->object_id, "world");
//    char p;
//#if(PUBLISH_OCTOMAP)
//    this->octomap_pub_clear.request.publish_bin_cloud.data = true;
//    this->octomap_pub_clear.request.clear_octomap_cloud.data = false;
//    if(this->client_pub_octomap.call(this->octomap_pub_clear))
//        cout << "Octomap published" << endl;
//    else
//        cout << "Failed to call octomap publish service" << endl;
//    sleep(1);
//#endif
//    cout << "Press p" << endl;
//    cin >> p;
//    getchar();

//    this->removeCollisionObject("world");
//    cout << "Press p" << endl;
//    cin >> p;
//    getchar();

//#endif

//    vector<double> current_jts = this->getCurrentJoints();
//    bool found_plan = false;
//    this->min_path_value = 1000000;// set to a large value for the minimum path
//    this->is_retrieval_plan = false;
//    this->is_drop_plan = false;

//    cout << "Attempt with gripper extended and suction bend" << endl;
//    this->gripper_suction_sys_control.data[0] = 1;
//    this->gripper_suction_sys_control.data[1] = 2;
//    this->gripper_suction_sys_control.data[2] = 0;
//    this->pub_planning_position.publish(this->gripper_suction_sys_control);

//    this->current_group = 3;

//    if(gripperMultipleAxisPlans(axis_array, inverted_axis_array, normal_array, centroid_array, current_jts, this->current_group))
//        found_plan = true;

//    if(found_plan)
//    {
//        this->current_group = this->selected_group;
//        geometry_msgs::Point centroid;
//        centroid.x = this->object_centroid.getX();
//        centroid.y = this->object_centroid.getY();
//        centroid.z = this->object_centroid.getZ();
//        if(this->attemptGripperPosePlan(this->plan_axis, this->plan_normal, centroid))
//            cout << "Found plan with selected axis and normal" << endl;

//        displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
//        int execution_result;
//#if(PUBLISH_OCTOMAP)
//        this->octomap_pub_clear.request.publish_bin_cloud.data = false;
//        this->octomap_pub_clear.request.clear_octomap_cloud.data = true;
////        this->octomap_pub_clear.request.flag.data = false;
////        if(client_clear_octomap.call(this->octomap_pub_clear))
//        if(client_pub_octomap.call(this->octomap_pub_clear))
//        {
//            cout << "Cleared octomap" << endl;
//        }
//        else
//            cout << "Failed to call octomap service" << endl;
//        sleep(2);
//#endif
//        if(executePlan(execution_result, this->my_plan_cartesian_goal))
//        {
//            res.success.data = true;

//            // Close gripper in rviz
//            this->gripper_suction_sys_control.data[0] = 2;
//            this->gripper_suction_sys_control.data[1] = 2;
//            this->gripper_suction_sys_control.data[2] = 0;
//            this->pub_planning_position.publish(this->gripper_suction_sys_control);

//            // close gripper in real hardware
//            this->gripper_suction_sys_control.data[2] = 1;
//            this->pub_planning_position.publish(this->gripper_suction_sys_control);

//            this->gripper_closed = true;
//            return true;
//        }
//        else
//        {
//            displayPlannerError(execution_result);
//            res.success.data = false;
//            return true;
//        }
//    }

//    cout << "No plans found with all methods" << endl;
//#if(PUBLISH_OCTOMAP)
//    this->octomap_pub_clear.request.publish_bin_cloud.data = false;
//    this->octomap_pub_clear.request.clear_octomap_cloud.data = true;
////        this->octomap_pub_clear.request.flag.data = false;
////        if(client_clear_octomap.call(this->octomap_pub_clear))
//    if(client_pub_octomap.call(this->octomap_pub_clear))
//    {
//        cout << "Cleared octomap" << endl;
//    }
//    else
//        cout << "Failed to call octomap service" << endl;
//    sleep(1);
//#endif
//    #if(ADD_COLLISION_OBJECT)
//        this->removeCollisionObject("world");
//    #endif

//    res.success.data = false;
//    return true;
//}

bool RobotArm::serviceGripperGeneralizedPlanCallback(iitktcs_msgs_srvs::GripperPoseGoal::Request &req,
                                                     iitktcs_msgs_srvs::GripperPoseGoal::Response &res)
{
    cout << "Reached service for Gripper pose planning" << endl;

    vector<tf::Vector3> axis_array, inverted_axis_array, normal_array, centroid_array;
    cout << "No of handles found: " << req.grasp_poses.size() << endl;
    int num_grasp_poses;
    if(req.grasp_poses.size() > 5)
        num_grasp_poses = 5;
    else
        num_grasp_poses = req.grasp_poses.size();

    tf::StampedTransform transform;
    transform = this->getTransform("camera_link", "world");

    bool mesh_cup = false;
    this->mesh_cup_gripper = false;
    double mesh_cup_dot = 0.0;
    if(req.object_id.data == 25)
    {
#if(PLAN_GRIPPER_MESH_CUP)
        mesh_cup = true;
#else
        res.success.data = false;
        return true;
#endif
    }

    for(int i=0; i<num_grasp_poses; i++)
    {
        iitktcs_msgs_srvs::grasp_pose grasp_pose = req.grasp_poses[i];
        tf::Vector3 minor_axis(grasp_pose.axis.x, grasp_pose.axis.y, grasp_pose.axis.z);
        tf::Vector3 normal(grasp_pose.normal.x, grasp_pose.normal.y, grasp_pose.normal.z);

        tf::Vector3 axis = minor_axis.cross(normal);
        tf::Vector3 inverted_axis(axis);
        inverted_axis *= -1;
        tf::Vector3 centroid(grasp_pose.centroid.x, grasp_pose.centroid.y, grasp_pose.centroid.z);

        if(!mesh_cup)
        {
            axis = transformPointVector(transform, axis, true);
            inverted_axis = transformPointVector(transform, inverted_axis, true);
            normal = transformPointVector(transform, normal, true);
            centroid = transformPointVector(transform, centroid);
        }
        else
        {
            if(grasp_pose.normal.z < 0.707)
                continue;
            else
            {
                this->mesh_cup_gripper = true;
                centroid.setZ(centroid.getZ()-0.03);// pick at distance of 3cm above mesh cup base
            }
        }

        cout << centroid.getX() << " " << centroid.getY() << " " << centroid.getZ() << endl;
        cout << axis.getX() << " " << axis.getY() << " " << axis.getZ() << endl;
        cout << normal.getX() << " " << normal.getY() << " " << normal.getZ() << endl;
        cout << "axis length: " << axis.length() << " normal length: " << normal.length() << endl;
        if(axis.length() < 0.0005 || normal.length() < 0.0005)
        {
            cout << "Axis or normal length approx zero" << endl;
            continue;
        }
        else if(centroid.getX() < 0.000001 && centroid.getY() < 0.000001 && centroid.getZ() < 0.000001)
        {
            cout << "centroid at origin" << endl;
            continue;
        }
        else if(normal.getZ() < 0.0)
        {
            cout << "normal is inverted" << endl;
            continue;
        }
        else
        {
            axis_array.push_back(axis);
            inverted_axis_array.push_back(inverted_axis);
            normal_array.push_back(normal);
            centroid_array.push_back(centroid);
        }
    }
    if(centroid_array.size() == 0)
    {
        res.success.data = false;
        return true;
    }

    this->object_id = req.object_id.data;
//    this->object_normal.setX(req.normal.x); this->object_normal.setY(req.normal.y); this->object_normal.setZ(req.normal.z);
//    this->object_centroid.setX(req.centroid.x); this->object_centroid.setY(req.centroid.y); this->object_centroid.setZ(req.centroid.z);
    this->object_attach_pose_world = req.object_pose;

#if(ADD_COLLISION_OBJECT)
//    this->addPrimitiveCollisionObject(this->object_attach_pose_world, this->object_id, "world");
    this->addCollisionObject(this->object_attach_pose_world, this->object_id, "world");
    char p;
#if(PUBLISH_OCTOMAP)
    this->octomap_pub_clear.request.publish_bin_cloud.data = true;
    this->octomap_pub_clear.request.clear_octomap_cloud.data = false;
    if(this->client_pub_octomap.call(this->octomap_pub_clear))
        cout << "Octomap published" << endl;
    else
        cout << "Failed to call octomap publish service" << endl;
    sleep(1);
#endif
    cout << "Press p" << endl;
    cin >> p;
    getchar();

    this->removeCollisionObject("world");
    cout << "Press p" << endl;
    cin >> p;
    getchar();

#endif

    vector<double> current_jts = this->getCurrentJoints();
    bool found_plan = false;
    this->min_path_value = 1000000;// set to a large value for the minimum path
    this->is_retrieval_plan = false;
    this->is_drop_plan = false;

    if(this->mesh_cup_gripper)
    {
        // Close gripper in rviz
        // keep suction straight
        this->gripper_suction_sys_control.data[0] = 2;
        this->gripper_suction_sys_control.data[1] = 0;
        this->gripper_suction_sys_control.data[2] = 0;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);
        this->current_group = 3;
    }
    else
    {
        cout << "Attempt with gripper extended and suction bend" << endl;
        this->gripper_suction_sys_control.data[0] = 1;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);
        this->current_group = 3;
    }

    if(gripperMultipleAxisPlans(axis_array, inverted_axis_array, normal_array, centroid_array, current_jts, this->current_group))
        found_plan = true;

    if(found_plan)
    {
        this->current_group = this->selected_group;
        geometry_msgs::Point centroid;
        centroid.x = this->object_centroid.getX();
        centroid.y = this->object_centroid.getY();
        centroid.z = this->object_centroid.getZ();
        if(this->attemptGripperPosePlan(this->plan_axis, this->plan_normal, centroid))
            cout << "Found plan with selected axis and normal" << endl;

        displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
        int execution_result;
#if(PUBLISH_OCTOMAP)
        this->octomap_pub_clear.request.publish_bin_cloud.data = false;
        this->octomap_pub_clear.request.clear_octomap_cloud.data = true;
//        this->octomap_pub_clear.request.flag.data = false;
//        if(client_clear_octomap.call(this->octomap_pub_clear))
        if(client_pub_octomap.call(this->octomap_pub_clear))
        {
            cout << "Cleared octomap" << endl;
        }
        else
            cout << "Failed to call octomap service" << endl;
        sleep(2);
#endif
        if(this->mesh_cup_gripper)
        {
            // close gripper in real hardware
            this->gripper_suction_sys_control.data[2] = 1;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->gripper_closed = true;
            cout << "Waiting 5 seconds for gripper to close" << endl;
            sleep(5.0);

            cout << "In rviz gripper extended and suction straight after reaching mesh cup" << endl;
            this->gripper_suction_sys_control.data[0] = 1;
            this->gripper_suction_sys_control.data[1] = 0;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);
        }

        if(executePlan(execution_result, this->my_plan_cartesian_goal))
        {
            res.success.data = true;

            if(!this->mesh_cup_gripper)
            {
                // Close gripper in rviz
                this->gripper_suction_sys_control.data[0] = 2;
                this->gripper_suction_sys_control.data[1] = 2;
                this->gripper_suction_sys_control.data[2] = 0;
                this->pub_planning_position.publish(this->gripper_suction_sys_control);
            }

            // close gripper in real hardware
            this->gripper_suction_sys_control.data[2] = 1;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);
            if(this->mesh_cup_gripper)
            {
                this->gripper_closed = false;
                cout << "Waiting 3 seconds for gripper to open for mesh cup" << endl;
            }
            else
            {
                this->gripper_closed = true;
                cout << "Waiting 3 seconds for gripper to close" << endl;
            }
            sleep(3);


            return true;
        }
        else
        {
            displayPlannerError(execution_result);
            res.success.data = false;
            return true;
        }
    }

    cout << "No plans found with all methods" << endl;
#if(PUBLISH_OCTOMAP)
    this->octomap_pub_clear.request.publish_bin_cloud.data = false;
    this->octomap_pub_clear.request.clear_octomap_cloud.data = true;
//        this->octomap_pub_clear.request.flag.data = false;
//        if(client_clear_octomap.call(this->octomap_pub_clear))
    if(client_pub_octomap.call(this->octomap_pub_clear))
    {
        cout << "Cleared octomap" << endl;
    }
    else
        cout << "Failed to call octomap service" << endl;
    sleep(1);
#endif
    #if(ADD_COLLISION_OBJECT)
        this->removeCollisionObject("world");
    #endif

    res.success.data = false;
    return true;
}

tf::StampedTransform RobotArm::getTransform(string source_frame, string target_frame)
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    return transform;
}

// Function to transform a point or vector from sensor frame to Robot world frame
// is_vector_transform = true. Then the point is assumed to be vector
// by default is_vector_transform = false. Computes transform for the point
tf::Vector3 RobotArm::transformPointVector(tf::StampedTransform &transform, tf::Vector3 &point,
                                           bool is_vector_transform)
{
    tf::Vector3 tf_pt = transform*point;

    tf::Vector3 tf_point = tf_pt;
    if(is_vector_transform)
    {
        tf::Vector3 tr_origin = transform.getOrigin();
        tf_point -= tr_origin;

        return tf_point;
    }
    else
    {
        return tf_point;
    }
}

bool RobotArm::suctionMultipleAxisPlans(tf::Vector3 &axis1, tf::Vector3 &axis2, tf::Vector3 &normal,
                                        geometry_msgs::Point &centroid, vector<double> &current_jts, int &group_id)
{
    char c;
    //    cout << "Attempt with axis 1" << endl;
    tf::Vector3 target_axis = axis1;
    bool found_plan = false;
    if(this->attemptSuctionPosePlan(target_axis, normal, centroid))
    {
        if(updateMinDistancePlan(current_jts))
        {
            this->plan_axis = target_axis;
            this->plan_normal = normal;
            this->selected_group = group_id;
            this->flag_plan_major_target_axis = true;
            this->plan_major_axis = target_axis;
            this->plan_minor_axis = target_axis.cross(normal);

            found_plan = true;
            cout << "Current group: " << group_id << endl;
        }
    }

//    cout << "Press enter" << endl;
//    cin >> c;
//    getchar();
//    cout << "Attempt with axis 2" << endl;
    target_axis = axis2;

    if(this->attemptSuctionPosePlan(target_axis, normal, centroid))
    {
        if(updateMinDistancePlan(current_jts))
        {
            this->plan_axis = target_axis;
            this->plan_normal = normal;
            this->selected_group = group_id;
            this->flag_plan_major_target_axis = false;
            this->plan_major_axis = target_axis;
            this->plan_minor_axis = target_axis.cross(normal);

            found_plan = true;
            cout << "Current group: " << group_id << endl;
        }
    }

//    cout << "Press enter" << endl;
//    cin >> c;
//    getchar();
    //    *********************************************************************** //
    //    cout << "Attempt with inverted axis 2" << endl;
    target_axis *= -1;

    if(this->attemptSuctionPosePlan(target_axis, normal, centroid))
    {
        if(updateMinDistancePlan(current_jts))
        {
            this->plan_axis = target_axis;
            this->plan_normal = normal;
            this->selected_group = group_id;
            this->flag_plan_major_target_axis = false;
            this->plan_major_axis = target_axis;
            this->plan_minor_axis = target_axis.cross(normal);

            found_plan = true;
            cout << "Current group: " << group_id << endl;
        }
    }

//    cout << "Press enter" << endl;
//    cin >> c;
//    getchar();
    //    cout << "Attempt with inverted axis 1" << endl;
    target_axis = axis1;
    target_axis *= -1;

    if(this->attemptSuctionPosePlan(target_axis, normal, centroid))
    {
        if(updateMinDistancePlan(current_jts))
        {
            this->plan_axis = target_axis;
            this->plan_normal = normal;
            this->selected_group = group_id;
            this->flag_plan_major_target_axis = true;
            this->plan_major_axis = target_axis;
            this->plan_minor_axis = target_axis.cross(normal);

            found_plan = true;
            cout << "Current group: " << group_id << endl;
        }
    }

//    cout << "Press enter" << endl;
//    cin >> c;
//    getchar();

    if(found_plan)
        return true;
    else
        return false;
}

bool RobotArm::gripperMultipleAxisPlans(vector<tf::Vector3> &axis1_array, vector<tf::Vector3> &axis2_array, vector<tf::Vector3> &normal_array,
                                        vector<tf::Vector3> &centroid_array, vector<double> &current_jts,
                                        int &group_id)

{
    bool found_plan = false;
    cout << "Axis1 array size: " << axis1_array.size() << endl;
    cout << "Axis2 array size: " << axis2_array.size() << endl;
    for(int i=0; i<axis1_array.size(); i++)
    {
        char c;
        cout << "Attempt with axis 1" << endl;
        tf::Vector3 target_axis = axis1_array[i];

        geometry_msgs::Point centroid;
        centroid.x = centroid_array[i].getX();
        centroid.y = centroid_array[i].getY();
        centroid.z = centroid_array[i].getZ();
        if(this->attemptGripperPosePlan(target_axis, normal_array[i], centroid))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal_array[i];
                this->selected_group = group_id;
                this->object_centroid = centroid_array[i];
                found_plan = true;

                cout << "Current group: " << group_id << endl;
            }
        }
        cout << "Press enter" << endl;
        cin >> c;
        getchar();
        cout << "Attempt with axis 2" << endl;
        target_axis = axis2_array[i];
        if(this->attemptGripperPosePlan(target_axis, normal_array[i], centroid))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal_array[i];
                this->selected_group = group_id;
                this->object_centroid = centroid_array[i];
                found_plan = true;

                cout << "Current group: " << group_id << endl;
            }
        }

        cout << "Press enter" << endl;
        cin >> c;
        getchar();
    }
    if(found_plan)
        return true;
    else
        return false;
}

bool RobotArm::attemptSuctionPosePlan(tf::Vector3 axis, tf::Vector3 normal, geometry_msgs::Point &centroid)
{
    // x-suction: +ve of object axis
    // y-suction: +ve of object normal
    // see the urdf model
    tf::Vector3 vect = axis.cross(normal);
    tf::Matrix3x3 rotation_matrix(axis.getX(), normal.getX(), vect.getX(),
                                  axis.getY(), normal.getY(), vect.getY(),
                                  axis.getZ(), normal.getZ(), vect.getZ());

    if(this->genCartesianPlanPose(centroid, rotation_matrix))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool RobotArm::attemptGripperPosePlan(tf::Vector3 axis, tf::Vector3 normal, geometry_msgs::Point &centroid)
{
    // x-gripper: -ve of object normal
    // y-gripper: +ve of object axis
    // see the urdf model
    normal *= -1;
    tf::Vector3 vect = normal.cross(axis);
    tf::Matrix3x3 rotation_matrix(normal.getX(), axis.getX(), vect.getX(),
                                  normal.getY(), axis.getY(), vect.getY(),
                                  normal.getZ(), axis.getZ(), vect.getZ());

    if(this->genCartesianPlanPose(centroid, rotation_matrix))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool RobotArm::updateMinDistancePlan(vector<double> current_jts)
{
    vector<double> plan_end_jts =
            this->my_plan_dummy.trajectory_.joint_trajectory.points.back().positions;

    // Check whether joint limits are exceeded
    double jts_max = 2*M_PI, jts_min = -2*M_PI;
    for(int i=0; i<NO_JOINTS; i++)
    {
        if(plan_end_jts[i] >= jts_max || plan_end_jts[i] <= jts_min)
        {
            cout << "Joint limit exceeded for joint " << i << ": " << plan_end_jts[i] << endl;
            return false;
        }
    }

//    cout << "current jts: " ;
//    for(int i=0; i<NO_JOINTS; i++)
//        cout << current_jts[i] << " ";
//    cout << endl;
//    cout << "plan end jts: " ;
//    for(int i=0; i<NO_JOINTS; i++)
//        cout << plan_end_jts[i] << " ";
//    cout << endl;

    double avg_pos_diff = 0;

    for(int i = 0; i < current_jts.size(); i++)
        avg_pos_diff += fabs(current_jts[i] - plan_end_jts[i]) * fabs(current_jts[i] - plan_end_jts[i]);
    avg_pos_diff = sqrt(avg_pos_diff);
    avg_pos_diff /= current_jts.size();

    cout << "Min: " << this->min_path_value << " Path difference: " << avg_pos_diff << endl;
    if(avg_pos_diff < this->min_path_value)
    {
        this->min_path_value = avg_pos_diff;
        this->my_plan_cartesian_goal.trajectory_ = this->my_plan_dummy.trajectory_;

        return true;
    }
    else
        return false;
}

bool RobotArm::serviceSuctionPoseGoalCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req,
                                              iitktcs_msgs_srvs::UR5PoseGoal::Response &res)
{
    this->current_joints.clear();
    this->current_joints = this->getCurrentJoints();
    // ask manish to send major axis in req.axis and minor axis in req.axis1
    cout << "Reached service for Suction pose planning" << endl;
    tf::Vector3 axis1, axis2;
    axis1.setX(req.axis.x); axis1.setY(req.axis.y); axis1.setZ(req.axis.z);
    axis2.setX(req.axis1.x); axis2.setY(req.axis1.y); axis2.setZ(req.axis1.z);

//    cout << "axis1: " << axis1.getX() << " " << axis1.getY() << " " << axis1.getZ() << endl;
//    cout << "axis2: " << axis2.getX() << " " << axis2.getY() << " " << axis2.getZ() << endl;

    this->bin_id = req.bin_id.data;
    this->object_id = req.object_id.data;
    this->object_normal.setX(req.normal.x); this->object_normal.setY(req.normal.y); this->object_normal.setZ(req.normal.z);
    this->object_centroid.setX(req.centroid.x); this->object_centroid.setY(req.centroid.y); this->object_centroid.setZ(req.centroid.z);
    this->object_attach_pose_world = req.object_pose;

    Eigen::Affine3d pose_matrix;
    tf::poseMsgToEigen(this->object_attach_pose_world, pose_matrix);
    this->object_model_centroid.setX(this->object_attach_pose_world.position.x);
    this->object_model_centroid.setY(this->object_attach_pose_world.position.y);
    this->object_model_centroid.setZ(this->object_attach_pose_world.position.z);
//    this->plan_major_axis.setX(pose_matrix(0,0));
//    this->plan_major_axis.setY(pose_matrix(1,0));
//    this->plan_major_axis.setZ(pose_matrix(2,0));
//    this->plan_minor_axis.setX(pose_matrix(0,1));
//    this->plan_minor_axis.setY(pose_matrix(1,1));
//    this->plan_minor_axis.setZ(pose_matrix(2,1));
    this->object_hold_centroid.setX(req.centroid.x);
    this->object_hold_centroid.setY(req.centroid.y);
    this->object_hold_centroid.setZ(req.centroid.z);

//    cout << "Length axis1: " << axis1.length() << endl;
//    cout << "Length axis2: " << axis2.length() << endl;
//    cout << "Length normal: " << this->object_normal.length() << endl;

    if(axis1.length() < 0.0005 || axis2.length() < 0.0005 || this->object_normal.length() < 0.0005)
    {
        cout << "Provided zero length vectors" << endl;
        res.success.data = false;
        return false;
    }
    if(fabs(this->object_centroid.getX()) < 0.000001 && fabs(this->object_centroid.getY()) < 0.000001 && fabs(this->object_centroid.getZ()) < 0.000001)
    {
        cout << "Provided centroid at origin" << endl;
        res.success.data = false;
        return false;
    }
    axis1.normalize(); axis2.normalize(); this->object_normal.normalize();

    plotAxisNormalCentroid(axis1, axis2, this->object_model_centroid);
//    cout << "Display major(red) and minor(green) axis marker" << endl;

    double value = this->object_normal.getZ();
#if(ADD_COLLISION_OBJECT)
//    this->addPrimitiveCollisionObject(this->object_attach_pose_world, this->object_id, "world");
    this->addCollisionObject(this->object_attach_pose_world, this->object_id,"world");
    char p;

#if(PUBLISH_OCTOMAP)
    if(this->object_id != 1)
    {
        this->octomap_pub_clear.request.publish_bin_cloud.data = true;
        this->octomap_pub_clear.request.clear_octomap_cloud.data = false;
        if(this->client_pub_octomap.call(this->octomap_pub_clear))
        {
            cout << "Published octomap" << endl;
        }
        else
            cout << "Failed to call octomap service" << endl;
        sleep(1);
    }
#endif
    cout << "Press p" << endl;
    cin >> p;
    getchar();

    this->removeCollisionObject("world");
#endif

    vector<double> current_jts = this->getCurrentJoints();
    bool found_plan = false;
    this->min_path_value = 1000000;// set to a large value for the minimum path
    this->gripper_closed = false;
    this->is_retrieval_plan = false;
    this->is_drop_plan = false;
    if(value < 0)
    {
        cout << "**** Normal provided in negative z-direction ****\n Aborted motion plan" << endl;
        res.success.data = false;
        if(this->object_id != 1)
        {
#if(PUBLISH_OCTOMAP)
            this->octomap_pub_clear.request.publish_bin_cloud.data = false;
            this->octomap_pub_clear.request.clear_octomap_cloud.data = true;

            if(client_pub_octomap.call(this->octomap_pub_clear))
            {
                cout << "Cleared octomap" << endl;
            }
            else
                cout << "Failed to call octomap service" << endl;
            sleep(1);
#endif
        }
        return true;
    }
    else
    {
        char c;
        // if the normal is 0 to 30 degrees from the z-axis
        // attempt first with straight suction position
        // attempt second with 45 degress mid suction position
        if(value <= 1.0 && value > 0.866)
        {
            cout << "Range: 1.0 to " << "0.866" << endl;
            cout << "Value: " << value << endl;

            // *********************************************************************** //
            cout << "Attempt with suction straight" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 0;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 0;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;

            // *********************************************************************** //
            cout << "Attempt with suction mid" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 1;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 1;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;
        }
        // *********************************************************************** //
        // if the normal is 30 to 60 degrees from the z-axis
        // attempt first with 45 degrees mid suction position
        // attempt second with bend suction position
        else if(value <= 0.866 && value > 0.5)
        {
            cout << "Range: " << "0.866" << " " << "0.5" << endl;
            cout << "Value: " << value << endl;

            // *********************************************************************** //
            cout << "Attempt with suction mid" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 1;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 1;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;

            // *********************************************************************** //
            cout << "Attempt with suction bend" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 2;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 2;
            //            cout << "End effector set: " << this->arm_group_bend->getEndEffectorLink() << endl;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;
        }
        // *********************************************************************** //
        //        if the normal is 70 to 90 degrees from the z-axis
        //        attempt first with bend suction position
        //        attempt second with 45 degrees mid suction position
        else if(value <= 0.5 && value >= 0.0)
        {
            cout << "Range: " << "0.5" << " " << "0.0" << endl;
            cout << "Value: " << value << endl;

            // *********************************************************************** //
            cout << "Attempt with suction bend" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 2;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 2;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;

            // *********************************************************************** //
            cout << "Attempt with suction mid" << endl;
            this->gripper_suction_sys_control.data[0] = 0;
            this->gripper_suction_sys_control.data[1] = 1;
            this->gripper_suction_sys_control.data[2] = 0;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);

            this->current_group = 1;
            //            cout << "End effector set: " << this->arm_group_mid->getEndEffectorLink() << endl;

            if(suctionMultipleAxisPlans(axis1, axis2, this->object_normal, req.centroid, current_jts, this->current_group))
                found_plan = true;
        }

        if(this->non_planning_item.size() > 0)
        {
            for(int i=0; i<this->non_planning_item.size(); i++)
            {
                if(this->object_id == this->non_planning_item[i])
                {
                    found_plan = false;
                    break;
                }
            }
        }
        this->got_protective_stop = false;
        this->protective_joint_target_moved = false;
        this->is_suction_plan = true;
        if(found_plan)
        {
            if(this->selected_group == 0)
            {
                this->gripper_suction_sys_control.data[0] = 0;
                this->gripper_suction_sys_control.data[1] = 0;
                this->gripper_suction_sys_control.data[2] = 0;
                this->pub_planning_position.publish(this->gripper_suction_sys_control);

                this->current_group = 0;
                cout << "Min path found using suction straight" << endl;
                if(this->attemptSuctionPosePlan(this->plan_axis, this->plan_normal, req.centroid));
//                    cout << "Found plan with selected axis and normal" << endl;
            }
            else if(this->selected_group == 1)
            {
                this->gripper_suction_sys_control.data[0] = 0;
                this->gripper_suction_sys_control.data[1] = 1;
                this->gripper_suction_sys_control.data[2] = 0;
                this->pub_planning_position.publish(this->gripper_suction_sys_control);

                this->current_group = 1;
                cout << "Min path found using suction mid" << endl;
                if(this->attemptSuctionPosePlan(this->plan_axis, this->plan_normal, req.centroid));
//                    cout << "Found plan with selected axis and normal" << endl;
            }
            else if(this->selected_group == 2)
            {
                this->gripper_suction_sys_control.data[0] = 0;
                this->gripper_suction_sys_control.data[1] = 2;
                this->gripper_suction_sys_control.data[2] = 0;
                this->pub_planning_position.publish(this->gripper_suction_sys_control);

                this->current_group = 2;
                cout << "Min path found using suction bend" << endl;
                if(this->attemptSuctionPosePlan(this->plan_axis, this->plan_normal, req.centroid));
//                    cout << "Found plan with selected axis and normal" << endl;
            }

            displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
            int execution_result;
#if(PUBLISH_OCTOMAP)
            this->octomap_pub_clear.request.publish_bin_cloud.data = false;
            this->octomap_pub_clear.request.clear_octomap_cloud.data = true;

            if(client_pub_octomap.call(this->octomap_pub_clear))
            {
                cout << "Cleared octomap" << endl;
            }
            else
                cout << "Failed to call octomap service" << endl;
            sleep(2);
#endif
            iitktcs_msgs_srvs::gripper_suction_controller vacuum_control;
            vacuum_control.request.vacuum_cleaner.data = 1;
            if(this->client_vacuum_cleaner_control.call(vacuum_control))
                cout << "Turned ON vacuum cleaner" << endl;
            else
                cout << "Failed to call service: " << this->client_vacuum_cleaner_control.getService() << endl;

            if(executePlan(execution_result, this->my_plan_cartesian_goal))
            {
                res.success.data = true;

                if(this->got_protective_stop)
                {
                    if(this->jointTargetGoal(this->current_joints))
                    {
                        this->protective_joint_target_moved = true;
                        cout << "Moved robot to its initial state" << endl;
                    }
                    else
                    {
                        this->protective_joint_target_moved = false;
                        cout << "Failed to move robot to its initial state" << endl;
                    }
                }
                return true;
            }
            else
            {
//                displayPlannerError(execution_result);
                res.success.data = false;

//                if(this->got_protective_stop)
//                {
//                    if(this->jointTargetGoal(this->current_joints))
//                    {
//                        this->protective_joint_target_moved = true;
//                        cout << "Moved robot to its initial state" << endl;
//                    }
//                    else
//                    {
//                        this->protective_joint_target_moved = false;
//                        cout << "Failed to move robot to its initial state" << endl;
//                    }
//                }

                vacuum_control.request.vacuum_cleaner.data = 0;
                if(this->client_vacuum_cleaner_control.call(vacuum_control))
                    cout << "Turned OFF vacuum cleaner" << endl;
                else
                    cout << "Failed to call service: " << this->client_vacuum_cleaner_control.getService() << endl;

                return true;
            }
        }
    }

    // repose plan by manish

    cout << "No plans found with all methods" << endl;
#if(PUBLISH_OCTOMAP)
    this->octomap_pub_clear.request.publish_bin_cloud.data = false;
    this->octomap_pub_clear.request.clear_octomap_cloud.data = true;

    if(client_pub_octomap.call(this->octomap_pub_clear))
    {
        cout << "Cleared octomap" << endl;
    }
    else
        cout << "Failed to call octomap service" << endl;

    sleep(1);
#endif

    res.success.data = false;
    return true;
}

double RobotArm::projection_distance(tf::Vector3 &normal_vector, tf::Vector3 &point_plane,
                                     tf::Vector3 &point)
{
    tf::Vector3 temp_vector;
    temp_vector.setX(point.getX() - point_plane.getX());
    temp_vector.setY(point.getY() - point_plane.getY());
    temp_vector.setZ(point.getZ() - point_plane.getZ());

    double dot_product;

    dot_product = temp_vector.dot(normal_vector);

    double dist;
    dist = dot_product/normal_vector.length();
    return dist;
}

// Service call back for retrieving robot from the bin
bool RobotArm::serviceRetrieveRobotCallback(iitktcs_msgs_srvs::Retrieval::Request &req,
                                            iitktcs_msgs_srvs::Retrieval::Response &res)
{
    cout << "Reached retrieve object service" << endl;
    this->is_suction_plan = false;
    // get object attach pose wrt to the end eef link to be attached
    Eigen::Affine3d object_pose_matrix;
    tf::poseMsgToEigen(this->object_attach_pose_world, object_pose_matrix);
    tf::Vector3 vector1(object_pose_matrix(0,0), object_pose_matrix(1,0), object_pose_matrix(2,0));
    tf::Vector3 vector2(object_pose_matrix(0,1), object_pose_matrix(1,1), object_pose_matrix(2,1));
    tf::Vector3 vector3(object_pose_matrix(0,2), object_pose_matrix(1,2), object_pose_matrix(2,2));
    tf::Vector3 position(object_pose_matrix(0,3), object_pose_matrix(1,3), object_pose_matrix(2,3));

    tf::StampedTransform transform;
    this->current_group = this->selected_group;
    if(this->selected_group == 0)
    {
        transform = this->getTransform("world","s_st_eef_link");
    }
    else if(this->selected_group == 1)
    {
        transform = this->getTransform("world","s_mid_eef_link");
    }
    else if(this->selected_group == 2)
    {
        transform = this->getTransform("world","s_bend_eef_link");
    }
    else if(this->selected_group == 3)
    {
        transform = this->getTransform("world","gripper_eef_link");
    }

    if(this->selected_group == 0 || this->selected_group == 1 || this->selected_group == 2)
    {
        vector1 = this->transformPointVector(transform, vector1, true);
        vector2 = this->transformPointVector(transform, vector2, true);
        vector3 = this->transformPointVector(transform, vector3, true);
        position = this->transformPointVector(transform, position, false);
        object_pose_matrix(0,0) = vector1.getX(); object_pose_matrix(0,1) = vector2.getX(); object_pose_matrix(0,2) = vector3.getX(); object_pose_matrix(0,3) = position.getX();
        object_pose_matrix(1,0) = vector1.getY(); object_pose_matrix(1,1) = vector2.getY(); object_pose_matrix(1,2) = vector3.getY(); object_pose_matrix(1,3) = position.getY();
        object_pose_matrix(2,0) = vector1.getZ(); object_pose_matrix(2,1) = vector2.getZ(); object_pose_matrix(2,2) = vector3.getZ(); object_pose_matrix(2,3) = position.getZ();
        object_pose_matrix(0,3) = 0.0;
        object_pose_matrix(1,3) = 0.0;
        object_pose_matrix(2,3) = 0.0;

        tf::poseEigenToMsg(object_pose_matrix, this->object_attach_pose_suction);
    }
    else if(this->selected_group == 3)
    {
        vector1 = this->transformPointVector(transform, vector1, true);
        vector2 = this->transformPointVector(transform, vector2, true);
        vector3 = this->transformPointVector(transform, vector3, true);
        position = this->transformPointVector(transform, position, false);
        object_pose_matrix(0,0) = vector1.getX(); object_pose_matrix(0,1) = vector2.getX(); object_pose_matrix(0,2) = vector3.getX(); object_pose_matrix(0,3) = position.getX();
        object_pose_matrix(1,0) = vector1.getY(); object_pose_matrix(1,1) = vector2.getY(); object_pose_matrix(1,2) = vector3.getY(); object_pose_matrix(1,3) = position.getY();
        object_pose_matrix(2,0) = vector1.getZ(); object_pose_matrix(2,1) = vector2.getZ(); object_pose_matrix(2,2) = vector3.getZ(); object_pose_matrix(2,3) = position.getZ();
        object_pose_matrix(0,3) = 0.0;
        object_pose_matrix(1,3) = 0.0;
        object_pose_matrix(2,3) = 0.0;

        tf::poseEigenToMsg(object_pose_matrix, this->object_attach_pose_gripper);
    }

#if(ADD_COLLISION_OBJECT)
    // remove the object
    this->removeCollisionObject("world");
#endif

    // retrieve plan
    tf::Vector3 vect, normal = this->plan_normal, axis = this->plan_axis;
    tf::Matrix3x3 rotation_matrix;

    if(this->selected_group == 0 || this->selected_group == 1 || this->selected_group == 2)
    {
        vect = axis.cross(normal); // see urdf model for suction
        rotation_matrix.setValue(axis.getX(), normal.getX(), vect.getX(),
                                 axis.getY(), normal.getY(), vect.getY(),
                                 axis.getZ(), normal.getZ(), vect.getZ());
    }
    else if(this->selected_group == 3)
    {
        normal *= -1; // see urdf model for gripper
        vect = normal.cross(axis);
        rotation_matrix.setValue(normal.getX(), axis.getX(), vect.getX(),
                                 normal.getY(), axis.getY(), vect.getY(),
                                 normal.getZ(), axis.getZ(), vect.getZ());
        normal *= -1;
    }
    Eigen::Affine3d trg_pose_matrix;
    for(int r=0; r<3; r++)
    {
        for(int c=0; c<3; c++)
        {
            trg_pose_matrix(r,c) = rotation_matrix[r][c];
        }
    }
    trg_pose_matrix(0,3) = this->object_centroid.getX(); trg_pose_matrix(1,3) = this->object_centroid.getY(); trg_pose_matrix(2,3) = this->object_centroid.getZ();
    trg_pose_matrix(3,0) = trg_pose_matrix(3,1) = trg_pose_matrix(3,2) = 0.0; trg_pose_matrix(3,3) = 1.0;

    tf::Vector3 centroid(trg_pose_matrix(0,3), trg_pose_matrix(1,3), trg_pose_matrix(2,3));

    plotAxisNormalCentroid(axis, normal, centroid);

    if(this->protective_joint_target_moved)
    {
        int count = 10;
        double flow_sensor_value = 0;
        cout << "Waiting for flow sensor reading" << endl;
#if(CHECK_FLOW_SENSOR)
        while(ros::ok() && count != 0)
        {
            this->flag_get_flow_sensor_reading = true;
            ros::spinOnce();
            if(!this->flag_get_flow_sensor_reading)
            {
                flow_sensor_value += this->flow_sensor_adc;
                count--;
            }
        }
#else
        flow_sensor_value = 250*10;
#endif

        cout << "Avg flow sensor: " << flow_sensor_value/10 << endl;
        cout << "Th flow sensor: " << FLOW_SENSOR_TH << endl;
        int avg_flow_sensor = flow_sensor_value/10;

        if(avg_flow_sensor < FLOW_SENSOR_TH)
        {
            res.object_present.data = true;
            res.success.data = true;
        }
        else
        {
            cout << "Object is not grasped" << endl;
            res.object_present.data = false;
            res.success.data = true;

            iitktcs_msgs_srvs::gripper_suction_controller vacuum_control;
            vacuum_control.request.vacuum_cleaner.data = 0;
            if(this->client_vacuum_cleaner_control.call(vacuum_control))
                cout << "Turned OFF vacuum cleaner" << endl;
            else
                cout << "Failed to call service: " << this->client_vacuum_cleaner_control.getService() << endl;
        }

        // for dropping non target item
        double default_upward_jts[][NO_JOINTS] = {{0.0890376, -1.55013, 1.77058, -1.82832, -1.59681, 0.0956652},
                                                  {-0.535401, -1.56516, 1.78588, -1.82982, -1.57258, -0.528362}
                                                 };
        if(res.object_present.data && req.change_bin.data)
        {
            cout << "Object is present, changing bin location" << endl;
            vector<double> goal(NO_JOINTS);
            if(req.bin_id.data == 1)
            {
                for(int j=0; j<NO_JOINTS; j++)
                    goal[j] = default_upward_jts[0][j];
            }
            else
            {
                for(int j=0; j<NO_JOINTS; j++)
                    goal[j] = default_upward_jts[1][j];
            }

            this->current_group = 0;
            int execution_status;
            this->arm_group_straight->setJointValueTarget(goal);

            *(this->plan_result) = this->arm_group_straight->plan(this->my_plan_pose_joint_goal);

            if(this->plan_result->val == 1)
            {
                displayTrajectoryPoints(this->my_plan_pose_joint_goal, "joint");
                if(executePlan(execution_status, this->my_plan_pose_joint_goal))
                {
                    cout << "Changed the bin location" << endl;
                }
                else
                {
                    cout << "Failed to change bin location " << endl;
                }
            }
            else
            {
                cout << "Failed to find change bin location plan" << endl;

                iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                if(this->client_checkclear_protective_stop.call(check_clear_ps))
                {
                    if(check_clear_ps.response.flag_protective_stopped.data)
                    {
                        cout << "Change bin: Robot was protective stopped and cleared" << endl;
                    }
                    else
                    {
                        cout << "Change bin: Robot is OK!!" << endl;
                    }
                }
                else
                    cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
            }
        }

        return true;
    }
    // make 5 attempts to find plan with retrieval pt above bin plane at a distance of multiple of 2cm along the normal
    for(int i=0; i<10; i++)
    {
        // store pose interms of waypoints for both the provided axis and with inverted axis.
        // here we assume that the normal is always opposite to the approach direction and protruding from object surface
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose pose;

        vector<tf::Vector3> path_array; // vector for plotting waypoints
        tf::Vector3 path_points;

        // at distance of multiples of 5cm in the direction of normal
        tf::Vector3 pt1 = this->getEndpt(normal, centroid, this->distance_along_normal*(i+1));

        this->nh.getParam("retrieve_pick_height", this->retrieve_pick_height);
//        double distance = projection_distance(this->bin_normal, centroid, pt1);
//        distance = fabs(distance);
//        double pick_height = distance+this->retrieve_pick_height;
        double step = 0.04;
        double pick_height = bin_centroid[0].getZ()-pt1.getZ() + this->retrieve_pick_height - i*step;
        cout << "Object height: " << pt1.getZ() << " bin centroid: " << bin_centroid[0].getZ() << endl;
        cout << "retrieve height: " << this->retrieve_pick_height - i*step << " pick height: " << pick_height << endl;
//        cout << "Reach height: " << pick_height << endl;
        tf::Vector3 dst_point = this->getEndpt(this->bin_normal, pt1, pick_height);
        cout << "Final Reach height: " << dst_point.getZ() << endl;
        tf::poseEigenToMsg(trg_pose_matrix, pose);

        pose.position.x = pt1.getX();
        pose.position.y = pt1.getY();
        pose.position.z = pt1.getZ();
        path_points.setX(pose.position.x);
        path_points.setY(pose.position.y);
        path_points.setZ(pose.position.z);
        path_array.push_back(path_points);
        waypoints.push_back(pose);

        // point above the bin
        pose.position.x = dst_point.getX();
        pose.position.y = dst_point.getY();
        pose.position.z = dst_point.getZ();
        path_points.setX(pose.position.x);
        path_points.setY(pose.position.y);
        path_points.setZ(pose.position.z);
        path_array.push_back(path_points);
        waypoints.push_back(pose);

        sphereMarker(path_array, this->pub_markerarray_centroid);
        this->is_retrieval_plan = true;
        this->is_drop_plan = false;
        if(this->genWayPointsPlan(waypoints))
        {
            int execution_result;
            this->my_plan_cartesian_goal.trajectory_ = this->my_plan_dummy.trajectory_;

            displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
            if(executePlan(execution_result, this->my_plan_cartesian_goal))
            {
#if(ADD_COLLISION_OBJECT && ATTACH_OBJECT)
                if(this->selected_group == 0)
                {
                    addCollisionObject(this->object_attach_pose_suction, this->object_id, "s_st_eef_link");
                    attachCollisionObject();
                }
                else if(this->selected_group == 1)
                {
                    addCollisionObject(this->object_attach_pose_suction, this->object_id, "s_mid_eef_link");
                    attachCollisionObject();
                }
                else if(this->selected_group == 2)
                {
                    addCollisionObject(this->object_attach_pose_suction, this->object_id, "s_bend_eef_link");
                    attachCollisionObject();
                }
//                else if(this->selected_group == 3)
//                {
//                    addCollisionObject(this->object_attach_pose_gripper, this->object_id, "gripper_eef_link");
//                    attachCollisionObject();
//                }
#endif
                int count = 10;
                double flow_sensor_value = 0;
                cout << "Waiting for flow sensor reading" << endl;
#if(CHECK_FLOW_SENSOR)
                while(ros::ok() && count != 0)
                {
                    this->flag_get_flow_sensor_reading = true;
                    ros::spinOnce();
                    if(!this->flag_get_flow_sensor_reading)
                    {
                        flow_sensor_value += this->flow_sensor_adc;
                        count--;
                    }
                }
#else
                flow_sensor_value = 250*10;
#endif

                cout << "Avg flow sensor: " << flow_sensor_value/10 << endl;
                cout << "Th flow sensor: " << FLOW_SENSOR_TH << endl;
                int avg_flow_sensor = flow_sensor_value/10;

                if(avg_flow_sensor < FLOW_SENSOR_TH)
                {
                    res.object_present.data = true;
                    res.success.data = true;
                }
                else
                {
                    cout << "Object is not grasped" << endl;
                    res.object_present.data = false;
                    res.success.data = true;

                    iitktcs_msgs_srvs::gripper_suction_controller vacuum_control;
                    vacuum_control.request.vacuum_cleaner.data = 0;
                    if(this->client_vacuum_cleaner_control.call(vacuum_control))
                        cout << "Turned OFF vacuum cleaner" << endl;
                    else
                        cout << "Failed to call service: " << this->client_vacuum_cleaner_control.getService() << endl;
                }

                // for dropping non target item
                double default_upward_jts[][NO_JOINTS] = {{0.0890376, -1.55013, 1.77058, -1.82832, -1.59681, 0.0956652},
                                                          {-0.535401, -1.56516, 1.78588, -1.82982, -1.57258, -0.528362}
                                                         };
                if(res.object_present.data && req.change_bin.data)
                {
                    cout << "Object is present, changing bin location" << endl;
                    vector<double> goal(NO_JOINTS);
                    if(req.bin_id.data == 1)
                    {
                        for(int j=0; j<NO_JOINTS; j++)
                            goal[j] = default_upward_jts[0][j];
                    }
                    else
                    {
                        for(int j=0; j<NO_JOINTS; j++)
                            goal[j] = default_upward_jts[1][j];
                    }

                    this->current_group = 0;
                    int execution_status;
                    this->arm_group_straight->setJointValueTarget(goal);

                    *(this->plan_result) = this->arm_group_straight->plan(this->my_plan_pose_joint_goal);

                    if(this->plan_result->val == 1)
                    {
                        displayTrajectoryPoints(this->my_plan_pose_joint_goal, "joint");
                        if(executePlan(execution_status, this->my_plan_pose_joint_goal))
                        {
                            cout << "Changed the bin location" << endl;
                        }
                        else
                        {
                            cout << "Failed to change bin location " << endl;
                        }
                    }
                    else
                    {
                        cout << "Failed to find change bin location plan" << endl;

                        iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                        if(this->client_checkclear_protective_stop.call(check_clear_ps))
                        {
                            if(check_clear_ps.response.flag_protective_stopped.data)
                            {
                                cout << "Change bin: Robot was protective stopped and cleared" << endl;
                            }
                            else
                            {
                                cout << "Change bin: Robot is OK!!" << endl;
                            }
                        }
                        else
                            cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
                    }
                }

                return true;
            }
            else
            {
                displayPlannerError(execution_result);

                iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                if(this->client_checkclear_protective_stop.call(check_clear_ps))
                {
                    if(check_clear_ps.response.flag_protective_stopped.data)
                    {
                        cout << "Retrieve robot was protective stopped and cleared" << endl;
                    }
                }
                else
                    cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
            }
        }
    }

    res.success.data = false;
    return true;
}

// service callback function to get forward kinematics for current or specified joint state
// of the robot
bool RobotArm::serviceGetFKCallback(iitktcs_msgs_srvs::GetFK::Request &req,
                                    iitktcs_msgs_srvs::GetFK::Response &res)
{
    cout << "Reached ur5_control forward kinematics service ";
    this->current_end_effector_link = req.end_effector_link.data;
    //    if(strcmp(this->current_end_effector_link.c_str(), "nozele") == 0)
    if(strcmp(this->current_end_effector_link.c_str(), "s_st_eef_link") == 0)
        this->current_group = 0;
    else if(strcmp(this->current_end_effector_link.c_str(), "s_mid_eef_link") == 0)
        this->current_group = 1;
    else if(strcmp(this->current_end_effector_link.c_str(), "s_bend_eef_link") == 0)
        this->current_group = 2;
    else if(strcmp(this->current_end_effector_link.c_str(), "gripper_eef_link") == 0)
        this->current_group = 3;
    else
    {
        cout << "Error in choosing arm group" << endl;
        return false;
    }

    Eigen::Affine3d end_effector_state;
    if(req.current_jts.data)// check if FK is to be found for the current jts of arm
    {
        cout << "current jts" << endl;
        end_effector_state = this->getFK();
    }
    else
    {
        cout << "specific jts" << endl;
        // if FK is to be found for different jts than the current jts then copy the jt position
        vector<double> jts;
        cout << "Requested FK for jts:" << endl;
        for(vector<double>::const_iterator it = req.joint_angles.data.begin(); it != req.joint_angles.data.end(); ++it)
        {
            cout << *it << " ";
            jts.push_back(*it);
        }

        end_effector_state = this->getFK(jts, false);
    }

    double eerot[9];
    eerot[0] = end_effector_state(0,0); eerot[1] = end_effector_state(0,1); eerot[2] = end_effector_state(0,2);
    eerot[3] = end_effector_state(1,0); eerot[4] = end_effector_state(1,1); eerot[5] = end_effector_state(1,2);
    eerot[6] = end_effector_state(2,0); eerot[7] = end_effector_state(2,1); eerot[8] = end_effector_state(2,2);

    tf::Matrix3x3 R(eerot[0], eerot[1], eerot[2],
                    eerot[3], eerot[4], eerot[5],
                    eerot[6], eerot[7], eerot[8]);
    tf::Quaternion q;
    R.getRotation(q);

    res.pose.orientation.x = q.getX();
    res.pose.orientation.y = q.getY();
    res.pose.orientation.z = q.getZ();
    res.pose.orientation.w = q.getW();

    res.pose.position.x = end_effector_state(0,3);
    res.pose.position.y = end_effector_state(1,3);
    res.pose.position.z = end_effector_state(2,3);

//    cout << res.pose.position.x << " " << res.pose.position.y << " " << res.pose.position.z<< endl;

    return true;
}

bool RobotArm::serviceTestCalibrationCallback(iitktcs_msgs_srvs::TestCalibration::Request &req,
                                              iitktcs_msgs_srvs::TestCalibration::Response &res)
{
    cout << "Reached service for testing calibration" << endl;
    string end_effector_name = req.end_effector_name.data;

    if(strcmp("suction_straight", end_effector_name.c_str()) == 0)
    {
        this->current_group = 0;
        Eigen::Affine3d pose = this->getFK();

        cout << "Eigen Matrix: \n" << pose(0,0) << " " << pose(0,1) << " " << pose(0,2)
             << "\n" << pose(1,0) << " " << pose(1,1) << " " << pose(1,2)
             << "\n" << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << endl;

        tf::Matrix3x3 rotation_matrix(pose(0,0), pose(0,1), pose(0,2),
                                      pose(1,0), pose(1,1), pose(1,2),
                                      pose(2,0), pose(2,1), pose(2,2));
//        req.position.x = pose(0,3);
//        req.position.y = pose(1,3);
//        req.position.z = pose(2,3) - 0.1;
//        cout << "Decreased by 10 cm end effector straight" << endl;
        if(this->genCartesianPlanPose(req.position, rotation_matrix))
        {
            this->my_plan_cartesian_goal.trajectory_ = this->my_plan_dummy.trajectory_;
            int execution_result;
            if(executePlan(execution_result, this->my_plan_cartesian_goal))
            {
                cout << "Success: Moved the arm to target positionn" << endl;
                return true;
            }
            else
            {
                displayPlannerError(execution_result);
                return false;
            }
        }
        else
        {
            cout << "Failed to move to target position" << endl;

            // return here itself false not going further because octomap is not there
            // collision is not detected when kinect is rotated bottom
            return false;
        }
    }
    else if(strcmp("suction_bend", end_effector_name.c_str()) == 0)
    {
        this->current_group = 2;
        Eigen::Affine3d pose = this->getFK();

        tf::Matrix3x3 rotation_matrix(pose(0,0), pose(0,1), pose(0,2),
                                      pose(1,0), pose(1,1), pose(1,2),
                                      pose(2,0), pose(2,1), pose(2,2));

        if(this->genCartesianPlanPose(req.position, rotation_matrix))
        {
            this->my_plan_cartesian_goal.trajectory_ = this->my_plan_dummy.trajectory_;
            int execution_result;
            if(executePlan(execution_result, this->my_plan_cartesian_goal))
            {
                cout << "Success: Moved the arm to target positionn" << endl;
                return true;
            }
            else
            {
                displayPlannerError(execution_result);
                return false;
            }
        }
        else
        {
            cout << "Failed to move to target position" << endl;

            // return here itself false not going further because octomap is not there
            // collision is not detected when kinect is rotated bottom
            return false;
        }
    }
    return true;
}

#if(SHOW_TRAJECTRY_POINTS)
// add spherical marker for displaying a point
void RobotArm::addSphereMarker(double *centroid, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id)
{
    //    static int id = 0;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "centroid_marker";
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
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration((double) MARKERS_DURATION);

    //    id++;
}

// Function to plot marker in Rviz given set of trajectory points
void RobotArm::sphereMarker(vector<tf::Vector3> &points, ros::Publisher &marker_pub)
{
    //    std::cout << "Publishing Markers" << std::endl;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker sphere_marker;

    visualization_msgs::MarkerArray markerArrayDelete;
    visualization_msgs::Marker marker_delete;

    static vector<int> id_vector;

    string frame = "/world";
    if(id_vector.size() > 0)
    {
//        cout << "NO of markers: " << id_vector.size() << " cleared" << endl;
        for(int i=0; i<id_vector.size(); i++)
        {
            string ns = "centroid_marker";
            deleteMarkers(marker_delete, frame, id_vector[i], ns, "sphere");
            markerArrayDelete.markers.push_back(marker_delete);
        }
        marker_pub.publish(markerArrayDelete);
        id_vector.clear();

//        sleep(1);
    }

    double color[3] = {1.0, 1.0, 0.0};

    double centroid[3];

    //    cout << "Sphere ref color: yellow" << endl;
    for(int j=0; j<points.size(); j++)
    {
        tf::Vector3 v = points[j];
        centroid[0] = v.getX(); centroid[1] = v.getY(); centroid[2] = v.getZ();
        string frame = "/world";

        addSphereMarker(centroid,color,sphere_marker,frame, j);
        markerArray.markers.push_back(sphere_marker);

        id_vector.push_back(j);
    }

    marker_pub.publish(markerArray);

    return;
}


void RobotArm::addArrowMarker(vector<tf::Vector3> &start_pt, vector<tf::Vector3> &end_pt, double *color,
                              visualization_msgs::Marker &marker, std::string &frame_id,
                              int id)
{
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "orient_vector";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    for(int i=0; i<start_pt.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = start_pt[i].getX(); pt.y = start_pt[i].getY(); pt.z = start_pt[i].getZ();

        marker.points.push_back(pt);// start_pt

        pt.x = end_pt[i].getX(); pt.y = end_pt[i].getY(); pt.z = end_pt[i].getZ();
        marker.points.push_back(pt);// end_pt
    }

    marker.scale.x = 0.0075;
    marker.scale.y = 0.0125;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration((double) MARKERS_DURATION);
}

void RobotArm::deleteMarkers(visualization_msgs::Marker &marker, std::string &frame_id, int id,
                             string namesp, string marker_type)
{
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = namesp;
    marker.id = id;
    if(strcmp(marker_type.c_str(), "sphere")==0)
            marker.type = visualization_msgs::Marker::SPHERE;
    else if(strcmp(marker_type.c_str(), "arrow")==0)
            marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
}

void RobotArm::arrowMarker(vector<tf::Vector3> &start_point, vector<tf::Vector3> &end_point,
                           ros::Publisher &marker_pub, double *line_color)
{
    static vector<int> id_vector;

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker arrow_marker;

    visualization_msgs::MarkerArray markerArrayDelete;
    visualization_msgs::Marker arrow_marker_delete;

    string frame = "/world";
    if(id_vector.size() > 0)
    {
        for(int i=0; i<id_vector.size(); i++)
        {
            deleteMarkers(arrow_marker_delete, frame, id_vector[i],"orient_vector", "arrow");
            markerArrayDelete.markers.push_back(arrow_marker_delete);
        }
        marker_pub.publish(markerArrayDelete);
        id_vector.clear();
    }

    int id = 0;
    id_vector.push_back(id);
    addArrowMarker(start_point, end_point, line_color, arrow_marker, frame, id++);
    markerArray.markers.push_back(arrow_marker);

    marker_pub.publish(markerArray);

    return;
}

void RobotArm::plotAxisNormalCentroid(tf::Vector3 &axis, tf::Vector3 &normal, tf::Vector3 &centroid)
{
    double length_vector = 0.15;

    tf::Vector3 normal_tip = getEndpt(normal, centroid, length_vector);
    tf::Vector3 axis_tip = getEndpt(axis, centroid, length_vector);

    vector<tf::Vector3> normal_vector_array, axis_vector_array, base_pt_array;

    normal_vector_array.push_back(normal_tip);
    axis_vector_array.push_back(axis_tip);
    base_pt_array.push_back(centroid);

    vector<tf::Vector3> target_centroid_array;
    target_centroid_array.push_back(centroid);

    tf::Vector3 pt = this->getEndpt(normal, centroid, 0.05);
    target_centroid_array.push_back(pt);

    double color[3];
    color[0] = 0.0; color[1] = 1.0; color[2] = 0.0;
    //    cout << "Green color: Normal vector" << endl;
    arrowMarker(base_pt_array, normal_vector_array, this->pub_markerarray_normal, color);

    color[0] = 1.0; color[1] = 0.0; color[2] = 0.0;
    //    cout << "Red color: Axis vector" << endl;
    arrowMarker(base_pt_array, axis_vector_array, this->pub_markerarray_axis, color);

    sphereMarker(target_centroid_array, this->pub_markerarray_centroid);
}

#endif

#if(NEW_COLLISION_MODELS)
// function to add collision plates for rack placed horizontal
void RobotArm::addPlates(int bin_target, vector<string> plate_ids)
{
    double plates_height = 0.2;

    if(bin_target==0)
        bin_target=0;
    else
        bin_target = bin_target*4;
    cout << endl << "bin_target" << bin_target <<endl ;
    ROS_INFO("fake Trajectory");
    this->display_trajectory.trajectory_start = this->my_plan_dummy.start_state_;
    this->display_trajectory.trajectory.push_back(this->my_plan_dummy.trajectory_);
    this->display_publisher.publish(this->display_trajectory);

    sleep(1);

    this->coll1->header.frame_id = this->arm_group_straight->getPlanningFrame();
    this->coll2->header.frame_id = this->arm_group_straight->getPlanningFrame();
    this->coll3->header.frame_id = this->arm_group_straight->getPlanningFrame();
    this->coll4->header.frame_id = this->arm_group_straight->getPlanningFrame();
    this->coll5->header.frame_id = this->arm_group_straight->getPlanningFrame();
    this->coll1->id = plate_ids[0];
    this->coll2->id = plate_ids[1];
    this->coll3->id = plate_ids[2];
    this->coll4->id = plate_ids[3];
    this->coll5->id = plate_ids[4];

    double thickness_plates = 0.007;

    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = thickness_plates;
    primitive1.dimensions[1] = this->bin_corners[(bin_target+0)].getY()-this->bin_corners[(bin_target+1)].getY();
    primitive1.dimensions[2] = plates_height;


    geometry_msgs::Pose box_pose1;
    box_pose1.orientation.w = 1.0;
    box_pose1.position.x =  this->bin_corners[(bin_target+0)].getX();
    box_pose1.position.y =  this->bin_corners[(bin_target+0)].getY() - ((this->bin_corners[(bin_target+0)].getY()-this->bin_corners[(bin_target+1)].getY())/2);
    box_pose1.position.z =  this->bin_corners[(bin_target+0)].getZ()-plates_height/2;


    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = thickness_plates;
    primitive2.dimensions[1] = this->bin_corners[(bin_target+2)].getY()-this->bin_corners[(bin_target+3)].getY();
    primitive2.dimensions[2] = plates_height;


    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x =  this->bin_corners[(bin_target+2)].getX();
    box_pose2.position.y =  this->bin_corners[(bin_target+2)].getY() - ((this->bin_corners[(bin_target+2)].getY()-this->bin_corners[(bin_target+3)].getY())/2);
    box_pose2.position.z =  this->bin_corners[(bin_target+2)].getZ() - plates_height/2;

    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = this->bin_corners[(bin_target+1)].getX()-this->bin_corners[(bin_target+3)].getX();
    primitive3.dimensions[1] = thickness_plates;
    primitive3.dimensions[2] = plates_height;


    geometry_msgs::Pose box_pose3;
    box_pose3.orientation.w = 1.0;
    box_pose3.position.x =  this->bin_corners[(bin_target+1)].getX()-((this->bin_corners[(bin_target+1)].getX()-this->bin_corners[(bin_target+3)].getX())/2);
    box_pose3.position.y =  this->bin_corners[(bin_target+1)].getY();
    box_pose3.position.z =  this->bin_corners[(bin_target+1)].getZ() - plates_height/2;

    shape_msgs::SolidPrimitive primitive4;
    primitive4.type = primitive4.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = this->bin_corners[(bin_target+0)].getX()-this->bin_corners[(bin_target+2)].getX();
    primitive4.dimensions[1] = thickness_plates;
    primitive4.dimensions[2] = plates_height;

    geometry_msgs::Pose box_pose4;
    box_pose4.orientation.w = 1.0;
    box_pose4.position.x =  this->bin_corners[(bin_target+0)].getX()- ((this->bin_corners[(bin_target+0)].getX()-this->bin_corners[(bin_target+2)].getX())/2);
    box_pose4.position.y =  this->bin_corners[(bin_target+0)].getY();
    box_pose4.position.z =   this->bin_corners[(bin_target+0)].getZ() - plates_height/2;
    shape_msgs::SolidPrimitive primitive5;
    geometry_msgs::Pose box_pose5;

    if(bin_target==0)
    {
        primitive5.type = primitive5.BOX;
        primitive5.dimensions.resize(3);
        primitive5.dimensions[0] = this->bin_corners[(bin_target+0)].getX()-this->bin_corners[(bin_target+2)].getX();
        primitive5.dimensions[1] = this->bin_corners[(bin_target+0)].getY()-this->bin_corners[(bin_target+5)].getY();
        primitive5.dimensions[2] = thickness_plates;

        box_pose5.orientation.w = 1.0;
        box_pose5.position.x = (this->bin_corners[(bin_target+0)].getX()+this->bin_corners[(bin_target+2)].getX())/2;
        box_pose5.position.y =  (this->bin_corners[(bin_target+0)].getY()+this->bin_corners[(bin_target+5)].getY())/2;
        box_pose5.position.z =   this->bin_corners[(bin_target+0)].getZ() - plates_height - 0.01;
    }

    this->coll1->primitives.push_back(primitive1);
    this->coll1->primitive_poses.push_back(box_pose1);
    this->coll1->operation = this->coll1->ADD;

    this->coll2->primitives.push_back(primitive2);
    this->coll2->primitive_poses.push_back(box_pose2);
    this->coll2->operation = this->coll2->ADD;

    this->coll3->primitives.push_back(primitive3);
    this->coll3->primitive_poses.push_back(box_pose3);
    this->coll3->operation = this->coll3->ADD;

    this->coll4->primitives.push_back(primitive4);
    this->coll4->primitive_poses.push_back(box_pose4);
    this->coll4->operation = this->coll4->ADD;

    this->collision_objects1.push_back(*this->coll1);
    this->collision_objects1.push_back(*this->coll2);
    this->collision_objects1.push_back(*this->coll3);
    this->collision_objects1.push_back(*this->coll4);

    if(bin_target==0)
    {
//        this->coll5->primitives.push_back(primitive5);
//        this->coll5->primitive_poses.push_back(box_pose5);
//        this->coll5->operation = this->coll5->ADD;
//        this->collision_objects1.push_back(*this->coll5);
    }
    this->planning_scene_interface->addCollisionObjects(this->collision_objects1);

    ROS_INFO("Add an object into the world");

}

#else
// function to add collision plates for rack placed vertical
void RobotArm::Addplates(int bin_target)
{

    if(bin_target==0)
        bin_target=0;
    else
        bin_target = bin_target*4;
    cout << endl << "bin_target" << bin_target <<endl ;
    ROS_INFO("fake Trajectory");
    this->display_trajectory.trajectory_start = this->my_plan_dummy.start_state_;
    this->display_trajectory.trajectory.push_back(this->my_plan_dummy.trajectory_);
    this->display_publisher.publish(this->display_trajectory);

    this->coll1->header.frame_id = arm_group_ptr->getPlanningFrame();
    this->coll2->header.frame_id = arm_group_ptr->getPlanningFrame();
    this->coll3->header.frame_id = arm_group_ptr->getPlanningFrame();
    this->coll4->header.frame_id = arm_group_ptr->getPlanningFrame();
    this->coll1->id = "plate1";
    this->coll2->id = "plate2";
    this->coll3->id = "plate3";
    this->coll4->id = "plate4";


    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.7;
    primitive1.dimensions[1] = this->bin_corners[(bin_target+0)].getY()-this->bin_corners[(bin_target+1)].getY();
    primitive1.dimensions[2] = 0.01;


    geometry_msgs::Pose box_pose1;
    box_pose1.orientation.w = 1.0;
    box_pose1.position.x =  this->bin_corners[(bin_target+0)].getX()+0.35;
    box_pose1.position.y =  this->bin_corners[(bin_target+0)].getY() - ((this->bin_corners[(bin_target+0)].getY()-this->bin_corners[(bin_target+1)].getY())/2);
    box_pose1.position.z =  this->bin_corners[(bin_target+0)].getZ();


    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.7;
    primitive2.dimensions[1] = this->bin_corners[(bin_target+2)].getY()-this->bin_corners[(bin_target+3)].getY();
    primitive2.dimensions[2] = 0.01;


    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x =  this->bin_corners[(bin_target+2)].getX()+0.35;
    box_pose2.position.y =  this->bin_corners[(bin_target+2)].getY() - ((this->bin_corners[(bin_target+2)].getY()-this->bin_corners[(bin_target+3)].getY())/2);
    box_pose2.position.z =  this->bin_corners[(bin_target+2)].getZ();

    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 0.7;
    primitive3.dimensions[1] = 0.01;
    primitive3.dimensions[2] = this->bin_corners[(bin_target+1)].getZ()-this->bin_corners[(bin_target+3)].getZ();


    geometry_msgs::Pose box_pose3;
    box_pose3.orientation.w = 1.0;
    box_pose3.position.x =  this->bin_corners[(bin_target+1)].getX()+0.35;
    box_pose3.position.y =  this->bin_corners[(bin_target+1)].getY();
    box_pose3.position.z = this->bin_corners[(bin_target+1)].getZ()-((this->bin_corners[(bin_target+1)].getZ()-this->bin_corners[(bin_target+3)].getZ())/2);

    shape_msgs::SolidPrimitive primitive4;
    primitive4.type = primitive4.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = 0.7;
    primitive4.dimensions[1] = 0.01;
    primitive4.dimensions[2] = this->bin_corners[(bin_target+0)].getZ()-this->bin_corners[(bin_target+2)].getZ();


    geometry_msgs::Pose box_pose4;
    box_pose4.orientation.w = 1.0;
    box_pose4.position.x =  this->bin_corners[(bin_target+0)].getX()+0.35;
    box_pose4.position.y =  this->bin_corners[(bin_target+0)].getY();
    box_pose4.position.z =  this->bin_corners[(bin_target+0)].getZ()- ((this->bin_corners[(bin_target+0)].getZ()-this->bin_corners[(bin_target+2)].getZ())/2);


    this->coll1->primitives.push_back(primitive1);
    this->coll1->primitive_poses.push_back(box_pose1);
    this->coll1->operation = this->coll1->ADD;

    this->coll2->primitives.push_back(primitive2);
    this->coll2->primitive_poses.push_back(box_pose2);
    this->coll2->operation = this->coll2->ADD;

    this->coll3->primitives.push_back(primitive3);
    this->coll3->primitive_poses.push_back(box_pose3);
    this->coll3->operation = this->coll3->ADD;

    this->coll4->primitives.push_back(primitive4);
    this->coll4->primitive_poses.push_back(box_pose4);
    this->coll4->operation = this->coll4->ADD;

    this->collision_objects1.push_back(*this->coll1);
    this->collision_objects1.push_back(*this->coll2);
    this->collision_objects1.push_back(*this->coll3);
    this->collision_objects1.push_back(*this->coll4);
    this->planning_scene_interface->addCollisionObjects(this->collision_objects1);


    ROS_INFO("Add an object into the world");

}
#endif

void RobotArm::addCollisionObject(geometry_msgs::Pose &object_pose, int object_id, string frame)
{
    string path = "package://iitktcs_robot_description";

    stringstream ss ;
    ss << path << "/meshes/stl/" << this->model_names[object_id-1] << ".stl";
    cout << "Adding object " << this->model_names[object_id-1] << " into the scene" << endl;

    const string obj_type_str = "/" + this->model_names[object_id-1] + "/object_type";

    int obj_type = 100;
//    string obj_type;
//    int string_length = 0;
//    while(string_length==0)
    while(obj_type == 100)
    {
        this->nh.getParam(obj_type_str, obj_type);
        cout << "Waiting for parameters: " << this->model_names[object_id-1].c_str() << endl;
//        string_length = obj_type.size();
    }

//    if(strcmp(obj_type.c_str(), "deformable") == 0)
//    {
//        this->added_object = false;
//        return;
//    }

    if(obj_type == 0 || obj_type == 100)
    {
        this->added_object = false;
        return;
    }

    ROS_INFO("fake Trajectory");
    this->my_plan_dummy.trajectory_.joint_trajectory.points.clear();

    this->display_trajectory.trajectory_start = this->my_plan_dummy.start_state_;
    this->display_trajectory.trajectory.push_back(this->my_plan_dummy.trajectory_);
    this->display_publisher.publish(this->display_trajectory);

    sleep(1);

    shapes::Mesh* m = shapes::createMeshFromResource(ss.str().c_str());
    //    ROS_INFO("mesh loaded");
    shapes::constructMsgFromShape(m, this->co_mesh_msg);
    this->co_mesh = boost::get<shape_msgs::Mesh>(this->co_mesh_msg);

    //**** specify the touch links ****//
    moveit_msgs::AttachedCollisionObject attached_object;

//    if(this->current_group == 0)
//        attached_object.link_name = frame;
//    else if(this->current_group == 1)
//        attached_object.link_name = frame;
//    else if(this->current_group == 2)
//        attached_object.link_name = frame;
    attached_object.link_name = frame;

    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = frame;
    /* The id of the object */
    attached_object.object.id = "ztobject";// this->model_names[object_id-1];

    attached_object.object.meshes.push_back(this->co_mesh);
    attached_object.object.mesh_poses.push_back(object_pose);
    for(int i=0; i<this->touch_links.size(); i++)
        attached_object.touch_links.push_back(this->touch_links[i]);
    attached_object.object.operation = attached_object.object.ADD;

    this->planning_scene.world.collision_objects.clear();
    this->planning_scene.world.collision_objects.push_back(attached_object.object);
    this->planning_scene.is_diff = true;
    this->planning_scene.robot_state.is_diff = true;
    //    this->planning_scene.allowed_collision_matrix = this->allowed_collision_matrix;
    this->planning_scene_diff_publisher.publish(this->planning_scene);
//    sleep(1.0);

//    ros::ServiceClient planning_scene_diff_client = this->nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
//    planning_scene_diff_client.waitForExistence();

//    moveit_msgs::ApplyPlanningScene srv;
//    srv.request.scene = this->planning_scene;
//    planning_scene_diff_client.call(srv);

    this->added_object = true;
}

void RobotArm::addPrimitiveCollisionObject(geometry_msgs::Pose &object_pose, int object_id,
                                           string frame)
{
    const string geometry_type_str = "/" + this->model_names[object_id-1] + "/geometry_type";
    const string dimensions_str = "/" + this->model_names[object_id-1] + "/dimensions";
    const string obj_type_str = "/" + this->model_names[object_id-1] + "/object_type";

    cout << "primitive " << geometry_type_str.c_str() << endl;
    vector<double> obj_dimensions;
    string obj_geometry_type, obj_type;

    while(obj_dimensions.size() == 0)
    {
        if(this->nh.getParam(geometry_type_str, obj_geometry_type))
            cout << "got param geometry" << endl;
        else
            cout << "didn't get param" << endl;
        this->nh.getParam(dimensions_str, obj_dimensions);
        this->nh.getParam(obj_type_str, obj_type);
        cout << "Waiting for parameters: " << this->model_names[object_id-1].c_str() << endl;
    }

    if(strcmp(obj_type.c_str(), "deformable") == 0)
    {
        this->added_object = false;
        return;
    }

    cout << "Adding Primtive for: " << this->model_names[object_id-1].c_str() << endl;
    shape_msgs::SolidPrimitive primitive;
    if(strcmp(obj_geometry_type.c_str(), "cubodial")==0)
    {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = obj_dimensions[0];
        primitive.dimensions[1] = obj_dimensions[1];
        primitive.dimensions[2] = obj_dimensions[2];
        cout << obj_dimensions[0] << " " << obj_dimensions[1] << " " << obj_dimensions[2] << endl;
    }
    else if(strcmp(obj_geometry_type.c_str(), "cylinder")==0)
    {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = obj_dimensions[0];
        primitive.dimensions[1] = obj_dimensions[1]/2;
    }

    ROS_INFO("fake Trajectory");
    this->my_plan_dummy.trajectory_.joint_trajectory.points.clear();
    this->display_trajectory.trajectory.clear();

    this->display_trajectory.trajectory_start = this->my_plan_dummy.start_state_;
    this->display_trajectory.trajectory.push_back(this->my_plan_dummy.trajectory_);
    this->display_publisher.publish(this->display_trajectory);

    sleep(1);

    //**** specify the touch links ****//
    moveit_msgs::AttachedCollisionObject attached_object;

//    if(this->current_group == 0)
//        attached_object.link_name = frame;
//    else if(this->current_group == 1)
//        attached_object.link_name = frame;
//    else if(this->current_group == 2)
//        attached_object.link_name = frame;
    attached_object.link_name = frame;

    /* The header must contain a valid TF frame*/
    attached_object.object.header.frame_id = frame;
    /* The id of the object */
    attached_object.object.id = "ztobject";

    attached_object.object.primitives.push_back(primitive);
//    object_pose.position.x = 1.0;
//    object_pose.position.y = 1.0;
//    object_pose.position.z = 1.0;
    cout << object_pose.position.x << " " << object_pose.position.y << " " << object_pose.position.z << endl;
    attached_object.object.primitive_poses.push_back(object_pose);
    for(int i=0; i<this->touch_links.size(); i++)
        attached_object.touch_links.push_back(this->touch_links[i]);
    attached_object.object.operation = attached_object.object.ADD;

//    this->planning_scene.world.collision_objects.clear();
//    this->planning_scene.world.collision_objects.push_back(attached_object.object);
//    this->planning_scene.is_diff = true;
//    this->planning_scene.robot_state.is_diff = true;
//    this->planning_scene_diff_publisher.publish(this->planning_scene);

    vector<moveit_msgs::CollisionObject> collision_object_array;
    collision_object_array.push_back(attached_object.object);
    this->planning_scene_interface->addCollisionObjects(collision_object_array);

    this->added_object = true;
}

void RobotArm::removeCollisionObject(string frame)
{
    if(this->added_object)
    {
//        moveit_msgs::CollisionObject remove_object;
//        remove_object.id = "ztobject";
//        remove_object.header.frame_id = frame;
//        remove_object.operation = remove_object.REMOVE;

//        ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
//        this->planning_scene.world.collision_objects.clear();
//        this->planning_scene.world.collision_objects.push_back(remove_object);
//        this->planning_scene.robot_state.attached_collision_objects.clear();
//        this->planning_scene_diff_publisher.publish(this->planning_scene);
        vector<string> object_ids;
        object_ids.push_back("ztobject");
        this->planning_scene_interface->removeCollisionObjects(object_ids);
        cout << "Removed collision object ztobject" << endl;
    }
    this->added_object = false;
}

void RobotArm::attachCollisionObject()
{
    if(this->added_object)
    {
        if(this->current_group == 0)
            this->arm_group_straight->attachObject("ztobject", this->arm_group_straight->getEndEffectorLink(), this->touch_links);
        else if(this->current_group == 1)
            this->arm_group_mid->attachObject("ztobject", this->arm_group_mid->getEndEffectorLink(), this->touch_links);
        else if(this->current_group == 2)
            this->arm_group_bend->attachObject("ztobject", this->arm_group_bend->getEndEffectorLink(), this->touch_links);
        else if(this->current_group == 3)
            this->arm_group_gripper->attachObject("ztobject", this->arm_group_gripper->getEndEffectorLink(), this->touch_links);
        this->is_attached_object_present = true;
    }
    else
        this->is_attached_object_present = false;
}

void RobotArm::detachCollisionObject()
{
    if(this->is_attached_object_present)
    {
        if(this->current_group == 0)
            this->arm_group_straight->detachObject("ztobject");
        else if(this->current_group == 1)
            this->arm_group_mid->detachObject("ztobject");
        else if(this->current_group == 2)
            this->arm_group_bend->detachObject("ztobject");
        else if(this->current_group == 3)
            this->arm_group_gripper->detachObject("ztobject");
        this->is_attached_object_present = false;
    }
}

void RobotArm::publishBox()

{
    //publish collision object
    moveit_msgs::CollisionObject object;
    object.header.frame_id = this->arm_group_straight->getEndEffectorLink();
    object.id = "transport_box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    object.primitives.push_back(primitive);
    geometry_msgs::Pose pose;
    pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
    pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0; pose.orientation.w = 1.0;

    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;// attached_object.object.ADD;

    moveit_msgs::PlanningScene planning_scene_add;
    planning_scene_add.world.collision_objects.push_back(object);
    planning_scene_add.is_diff = true;
    this->planning_scene_diff_publisher.publish(planning_scene_add);

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    //wait until box is published (I don't think that this is really needed)

    bool object_in_world = false;
    while(!object_in_world)
    {
        ROS_ERROR("waiting for box to appear");
        if(this->client_get_planning_scene.call(srv))
        {
            for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
            {
                if (srv.response.scene.world.collision_objects[i].id == "transport_box")
                {
                    cout << "Object in world" << endl;
                    object_in_world = true;
                    break;
                }
            }
        }
    }

    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::GetPlanningScene scene_srv;

    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    if(!this->client_get_planning_scene.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
    }
    else
    {
        ROS_INFO_STREAM("Initial scene!");
        currentScene = scene_srv.response.scene;

        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        ROS_ERROR_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
        ROS_ERROR_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());
        ROS_ERROR_STREAM("size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size());

        for(int i=0; i<currentACM.entry_names.size(); i++)
        {
            cout << currentACM.entry_names[i] << ": ";
            for(int j=0; j<currentACM.entry_values[i].enabled.size(); j++)
            {
                if(currentACM.entry_values[i].enabled[j])
                    cout << "true ";
                else
                    cout << "false ";
            }
            cout << endl;
        }

        currentACM.entry_names.push_back("transport_box");

        moveit_msgs::AllowedCollisionEntry entry;
        entry.enabled.resize(currentACM.entry_names.size());

        for(int i = 0; i < entry.enabled.size(); i++)
            entry.enabled[i] = true;

        //add new row to allowed collsion matrix

        currentACM.entry_values.push_back(entry);
        for(int i = 0; i < currentACM.entry_values.size(); i++)
        {
            //extend the last column of the matrix
            currentACM.entry_values[i].enabled.push_back(true);
        }

        newSceneDiff.is_diff = true;
        newSceneDiff.allowed_collision_matrix = currentACM;

        this->planning_scene_diff_publisher.publish(newSceneDiff);
    }

    if(!this->client_get_planning_scene.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
    }
    else
    {
        ROS_INFO_STREAM("Modified scene!");
        currentScene = scene_srv.response.scene;
        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        ROS_ERROR_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());
        ROS_ERROR_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());
        ROS_ERROR_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());

        for(int i=0; i<currentACM.entry_names.size(); i++)
        {
            cout << currentACM.entry_names[i] << ": ";
            for(int j=0; j<currentACM.entry_values[i].enabled.size(); j++)
            {
                if(currentACM.entry_values[i].enabled[j])
                    cout << "true ";
                else
                    cout << "false ";
            }
            cout << endl;
        }
    }

    this->arm_group_straight->attachObject("transport_box", this->arm_group_straight->getEndEffectorLink(),
                                           this->touch_links);
}

bool RobotArm::getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix &current_acm)
{
    moveit_msgs::GetPlanningScene scene_srv;
    moveit_msgs::PlanningScene currentScene;

    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    if(!this->client_get_planning_scene.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Modified scene!");
        currentScene = scene_srv.response.scene;
        current_acm = currentScene.allowed_collision_matrix;

        for(int i=0; i<current_acm.entry_names.size(); i++)
        {
            cout << current_acm.entry_names[i] << ": ";
            for(int j=0; j<current_acm.entry_values[i].enabled.size(); j++)
            {
                if(current_acm.entry_values[i].enabled[j])
                    cout << "true ";
                else
                    cout << "false ";
            }
            cout << endl;
        }

        return true;
    }
}

void RobotArm::updatePlatesACM(moveit_msgs::AllowedCollisionMatrix &currentACM, vector<string> box1,
                               vector<string> box2)
{
    int total_new_collision_objects = box1.size()+box2.size();
    int size_robot_links = currentACM.entry_names.size();
    int size_after_plates = total_new_collision_objects+size_robot_links;

    for(int i=0; i<box1.size(); i++)
    {
        currentACM.entry_names.push_back(box1[i]);
    }
    for(int i=0; i<box2.size(); i++)
    {
        currentACM.entry_names.push_back(box2[i]);
    }

    moveit_msgs::AllowedCollisionEntry plates_entry;

    plates_entry.enabled.resize(currentACM.entry_names.size());

    for(int j=0; j<size_robot_links; j++) // till the robot links
        plates_entry.enabled[j] = false;
    for(int j=size_robot_links; j<size_after_plates-1; j++)// for only the plates
        plates_entry.enabled[j] = true;

    // collision disabled with the object
    plates_entry.enabled[size_after_plates-1] = false; // for the object

    for(int i=0; i<total_new_collision_objects; i++)
        currentACM.entry_values.push_back(plates_entry);// add collision entry for all the plates

    // update collision entry for the robot links for collision with new plates and object
    for(int i = 0; i < size_robot_links; i++)
    {
        // for robot links update the collision with plates and object
        for(int j=0; j<total_new_collision_objects; j++)
            currentACM.entry_values[i].enabled.push_back(false);
    }
    cout << "**** New matrix size : "<< currentACM.entry_names.size() << " " << currentACM.entry_values[0].enabled.size() << endl;

    for(int i=0; i<currentACM.entry_names.size(); i++)
    {
        cout << currentACM.entry_names[i] << ": ";
        for(int j=0; j<currentACM.entry_values[i].enabled.size(); j++)
        {
            if(currentACM.entry_values[i].enabled[j])
                cout << "true ";
            else
                cout << "false ";
        }
        cout << endl;
    }
}

void RobotArm::updateTargetObjectACM(moveit_msgs::AllowedCollisionMatrix &currentACM, string object_name)
{
    int total_new_collision_objects = 1;
    int size_after_plates = currentACM.entry_names.size();
    int size_after_objects = total_new_collision_objects+size_after_plates;

    currentACM.entry_names.push_back(object_name);

    moveit_msgs::AllowedCollisionEntry object_entry;

    object_entry.enabled.resize(currentACM.entry_names.size());

    for(int j=0; j<size_after_objects-1; j++)// till robot links + plates
        object_entry.enabled[j] = false;
    object_entry.enabled[size_after_objects-1] = true; // self collision with the object is true

    currentACM.entry_values.push_back(object_entry);// add collision entry for the object

    // update collision entry for the robot links for collision with new plates and object
    for(int i = 0; i < size_after_plates; i++)
    {
        // for robot links update the collision with plates and object
        for(int j=0; j<total_new_collision_objects; j++)
            currentACM.entry_values[i].enabled.push_back(false);
    }

    cout << "**** New matrix size : "<< currentACM.entry_names.size() << " " << currentACM.entry_values[0].enabled.size() << endl;
    for(int i=0; i<currentACM.entry_names.size(); i++)
    {
        cout << currentACM.entry_names[i] << ": ";
        for(int j=0; j<currentACM.entry_values[i].enabled.size(); j++)
        {
            if(currentACM.entry_values[i].enabled[j])
                cout << "true ";
            else
                cout << "false ";
        }
        cout << endl;
    }
}

bool RobotArm::executePlan(int &status_code, moveit::planning_interface::MoveGroup::Plan &plan,
                           bool wait_to_end)
{
    char p='p';
//    cout << "press p to move arm" << endl;
//    cin >> p;
    fflush(stdin);
    // extract the final jts
    vector<double> final_jts = plan.trajectory_.joint_trajectory.points.back().positions;
    bool executed_plan = false;

    if(p == 'p')
    {
        // publish to execute the position of gripper and suction on real hardware
        if(!this->gripper_closed)
        {
            this->gripper_suction_sys_control.data[2] = 1;
            this->pub_planning_position.publish(this->gripper_suction_sys_control);
        }

        bool execute = true;
        moveit::planning_interface::MoveItErrorCode execution_status;
        if(this->current_group == 0)
            execution_status = this->arm_group_straight->execute(plan);
        else if(this->current_group == 1)
            execution_status = this->arm_group_mid->execute(plan);
        else if(this->current_group == 2)
            execution_status = this->arm_group_bend->execute(plan);
        else if(this->current_group == 3)
            execution_status = this->arm_group_gripper->execute(plan);
        else
        {
            cout << "Error in choosing arm group: " << this->current_group << " is current group" << endl;
            executed_plan = false;
            execute = false;
        }

        if(execute)
        {
            status_code = execution_status.val;

            if(status_code==1)
            {
                if(wait_to_end)
                {
                    if(checkJointsStateReached(final_jts))
                    {
                        cout << "Plan executed" << endl;
                        executed_plan = true;
                    }
                    else
                    {
                        executed_plan = false;
                    }
                }
                else
                {
                    cout << "Plan executed" << endl;
                    executed_plan = true;
                }

                iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                if(this->client_checkclear_protective_stop.call(check_clear_ps))
                {
                    if(check_clear_ps.response.flag_protective_stopped.data)
                    {
                        this->got_protective_stop = true;
                        cout << "Robot was protective stopped and cleared" << endl;
                        executed_plan = true;
                    }
                }
                else
                    cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
            }
            else
            {
                displayPlannerError(status_code);
                executed_plan = false;

                if(this->is_suction_plan)
                {
                    iitktcs_msgs_srvs::CheckClearProtectiveStop check_clear_ps;
                    if(this->client_checkclear_protective_stop.call(check_clear_ps))
                    {
                        if(check_clear_ps.response.flag_protective_stopped.data)
                        {
                            this->got_protective_stop = true;
                            cout << "Robot was protective stopped and cleared" << endl;
                            executed_plan = true;
                        }
                        else
                        {
                            cout << "Robot is OK!!, some other failure in execution" << endl;
                        }
                    }
                    else
                        cout << "Failed to call service: " << this->client_checkclear_protective_stop.getService() << endl;
                }
            }

            if(executed_plan)
                return true;
            else
                return false;
        }
        else
            return false;
    }
    else
    {
        return true;
    }

}

// service for checking position of gripper and suction position in rviz and in real gripper system
bool RobotArm::serviceGripperRvizRealCallback(iitktcs_msgs_srvs::GripperRealRviz::Request &req,
                                              iitktcs_msgs_srvs::GripperRealRviz::Response &res)
{
    if(req.select.data == 0)//suction straight
    {
        cout << "suction straight" << endl;
        this->current_group = 0;
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 0;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }
    if(req.select.data == 1)//suction mid
    {
        cout << "suction mid" << endl;
        this->current_group = 1;
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 1;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }
    if(req.select.data == 2)//suction bend
    {
        cout << "suction bend" << endl;
        this->current_group = 2;
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }

    if(req.select.data == 20)//gripper retracted
    {
        cout << "gripper retracted suction bend" << endl;
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }
    if(req.select.data == 21)//gripper extended
    {
        cout << "gripper extended suction bend" << endl;
        this->gripper_suction_sys_control.data[0] = 1;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }
    if(req.select.data == 22)//gripper extended
    {
        cout << "gripper closed suction bend" << endl;
        this->gripper_suction_sys_control.data[0] = 2;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }

    if(req.select.data == 10)//execute
    {
        cout << "Executing command" << endl;
        //        this->gripper_suction_sys_control.data[0] = 0;
        //        this->gripper_suction_sys_control.data[1] = 0;
        this->gripper_suction_sys_control.data[2] = 1;

        this->pub_planning_position.publish(this->gripper_suction_sys_control);
    }
    return true;

}

bool RobotArm::serviceValleyPlanningCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req, iitktcs_msgs_srvs::UR5PoseGoal::Response &res)
{
    cout << "Reached service for dropping object using valley detection pose" << endl;
    if(!this->gripper_closed)
    {
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 0;
        this->gripper_suction_sys_control.data[2] = 0;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);

        this->gripper_suction_sys_control.data[2] = 1;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);
//        addCollisionObject(this->object_attach_pose_suction, this->object_id, "s_st_eef_link");
//        attachCollisionObject();
    }
    else
    {
//        addCollisionObject(this->object_attach_pose_gripper, this->object_id, "gripper_eef_link");
//        attachCollisionObject();
    }

    tf::Vector3 valley_major_axis(req.axis.x, req.axis.y, req.axis.z);
    tf::Vector3 valley_minor_axis(req.axis1.x, req.axis1.y, req.axis1.z);
    tf::Vector3 valley_centroid(req.centroid.x, req.centroid.y, req.centroid.z);
    cout << "Centroid: " << req.centroid.x << " " << req.centroid.y << " " << req.centroid.z << endl;
    cout << "Major Axis : " << req.axis.x << " " << req.axis.y << " " << req.axis.z << endl;
    cout << "Minor Axis : " << req.axis1.x << " " << req.axis1.y << " " << req.axis1.z << endl;
    if(valley_major_axis.length() < 0.0005 || valley_minor_axis.length() < 0.0005)
    {
        cout << "Provided zero length vectors" << endl;
        res.success.data = false;
        return false;
    }

    if(fabs(valley_centroid.getX()) < 0.000001 && fabs(valley_centroid.getY()) < 0.000001 && fabs(valley_centroid.getZ() < 0.000001))
    {
        cout << "Provided centroid at origin" << endl;
        res.success.data = false;
        return false;
    }

    this->min_path_value = 1000000;
    this->is_drop_plan = true;

    tf::StampedTransform transform = this->getTransform("camera_link", "world");
    valley_major_axis = this->transformPointVector(transform, valley_major_axis, true);
    valley_minor_axis = this->transformPointVector(transform, valley_minor_axis, true);
    valley_centroid = this->transformPointVector(transform, valley_centroid);

    plotAxisNormalCentroid(valley_major_axis, valley_minor_axis, valley_centroid);

    // increase the drop height by 12cm
    valley_centroid.setZ(valley_centroid.getZ()+0.12);

    vector<tf::Vector3> valley_axes, object_planning_axes;
    valley_axes.push_back(valley_major_axis);
    valley_axes.push_back(valley_minor_axis);
    object_planning_axes.push_back(this->plan_major_axis);
    object_planning_axes.push_back(this->plan_minor_axis);

    vector<double> current_jts = this->getCurrentJoints();
    bool found_plan = false;
    if(this->current_group == 0 || this->current_group == 1 || this->current_group == 2)
    {
        cout << "Attempt with suction straight" << endl;
        this->gripper_suction_sys_control.data[0] = 0;
        this->gripper_suction_sys_control.data[1] = 0;
        this->gripper_suction_sys_control.data[2] = 0;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);

        this->current_group = 0;
        geometry_msgs::Point drop_pt1 = this->alignAxes(valley_axes, object_planning_axes, valley_centroid, this->object_hold_centroid, this->object_model_centroid);

        tf::Vector3 target_axis, normal;
        // axis inside attemptSuctionPosePlan sholud be target axis
        // arrange accordingly
        if(this->flag_plan_major_target_axis)
        {
            target_axis = valley_axes[0];
            normal = valley_axes[1].cross(target_axis);
        }
        else
        {
            target_axis = valley_axes[1];
            normal = valley_axes[0].cross(target_axis);
        }

        if(this->attemptSuctionPosePlan(target_axis, normal, drop_pt1))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal;
                this->selected_group = 0;
                this->object_centroid.setX(drop_pt1.x);
                this->object_centroid.setY(drop_pt1.y);
                this->object_centroid.setZ(drop_pt1.z);
                found_plan = true;
            }
        }

        geometry_msgs::Point drop_pt2 = this->alignAxes(valley_axes, object_planning_axes, valley_centroid, this->object_hold_centroid, this->object_model_centroid, true);

        if(this->flag_plan_major_target_axis)
        {
            target_axis = valley_axes[0];
            normal = valley_axes[1].cross(target_axis);
        }
        else
        {
            target_axis = valley_axes[1];
            normal = valley_axes[0].cross(target_axis);
        }

        if(this->attemptSuctionPosePlan(target_axis, normal, drop_pt2))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal;
                this->selected_group = 0;
                this->object_centroid.setX(drop_pt2.x);
                this->object_centroid.setY(drop_pt2.y);
                this->object_centroid.setZ(drop_pt2.z);
                found_plan = true;
            }
        }
    }
    else if(this->current_group == 3)
    {
        cout << "Attempt with gripper extended and suction bend" << endl;
        this->gripper_suction_sys_control.data[0] = 1;
        this->gripper_suction_sys_control.data[1] = 2;
        this->gripper_suction_sys_control.data[2] = 0;
        this->pub_planning_position.publish(this->gripper_suction_sys_control);

        geometry_msgs::Point drop_pt1 = this->alignAxes(valley_axes, object_planning_axes, valley_centroid, this->object_hold_centroid, this->object_model_centroid);

        tf::Vector3 target_axis, normal;
        // axis inside attemptGripperPosePlan sholud be target axis
        // arrange accordingly
        if(this->flag_plan_major_target_axis)
        {
            target_axis = valley_axes[0];
            normal = target_axis.cross(valley_axes[1]);
            normal *= -1;
        }
        else
        {
            target_axis = valley_axes[1];
            normal = target_axis.cross(valley_axes[0]);
            normal *= -1;
        }

        if(this->attemptGripperPosePlan(target_axis, normal, drop_pt1))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal;
                this->selected_group = 3;
                this->object_centroid.setX(drop_pt1.x);
                this->object_centroid.setY(drop_pt1.y);
                this->object_centroid.setZ(drop_pt1.z);
                found_plan = true;
            }
        }

        geometry_msgs::Point drop_pt2 = this->alignAxes(valley_axes, object_planning_axes, valley_centroid, this->object_hold_centroid, this->object_model_centroid, true);

        if(this->flag_plan_major_target_axis)
        {
            target_axis = valley_axes[0];
            normal = target_axis.cross(valley_axes[1]);
            normal *= -1;
        }
        else
        {
            target_axis = valley_axes[1];
            normal = target_axis.cross(valley_axes[0]);
            normal *= -1;
        }
        if(this->attemptGripperPosePlan(target_axis, normal, drop_pt2))
        {
            if(updateMinDistancePlan(current_jts))
            {
                this->plan_axis = target_axis;
                this->plan_normal = normal;
                this->selected_group = 3;
                this->object_centroid.setX(drop_pt2.x);
                this->object_centroid.setY(drop_pt2.y);
                this->object_centroid.setZ(drop_pt2.z);
                found_plan = true;
            }
        }
    }

    this->detachCollisionObject();

    if(found_plan)
    {
        if(this->selected_group==0)
        {
            this->removeCollisionObject("s_st_eef_link");

            this->current_group = this->selected_group;
            geometry_msgs::Point centroid;
            centroid.x = this->object_centroid.getX();
            centroid.y = this->object_centroid.getY();
            centroid.z = this->object_centroid.getZ();
            if(this->attemptSuctionPosePlan(this->plan_axis, this->plan_normal, centroid))
            {
                cout << "Found valley plan for drop using suction" << endl;
                this->is_drop_plan = false; // for next pick or single goal planning
                displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
                int execution_result;
                if(executePlan(execution_result, this->my_plan_cartesian_goal))
                {
                    res.success.data = true;
                    return true;
                }
                else
                {
                    displayPlannerError(execution_result);
                    res.success.data = false;
                    return true;
                }
            }
            else
            {
                this->is_drop_plan = false; // for next pick or single goal planning
                cout << "Failed to find plan" << endl;
                res.success.data = false;
                return true;
            }
        }
        else if(this->selected_group == 3)
        {
            this->removeCollisionObject("gripper_eef_link");

            this->current_group = this->selected_group;
            geometry_msgs::Point centroid;
            centroid.x = this->object_centroid.getX();
            centroid.y = this->object_centroid.getY();
            centroid.z = this->object_centroid.getZ();
            if(this->attemptGripperPosePlan(this->plan_axis, this->plan_normal, centroid))
            {
                cout << "Found valley plan for drop using gripper" << endl;
                this->is_drop_plan = false; // for next pick or single goal planning
                displayTrajectoryPoints(this->my_plan_cartesian_goal, "cartesian");
                int execution_result;
                if(executePlan(execution_result, this->my_plan_cartesian_goal))
                {
                    res.success.data = true;

                    // Open gripper in rviz
                    this->gripper_suction_sys_control.data[0] = 0;
                    this->gripper_suction_sys_control.data[1] = 2;
                    this->gripper_suction_sys_control.data[2] = 0;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);

                    // Open gripper in real hardware
                    this->gripper_suction_sys_control.data[2] = 1;
                    this->pub_planning_position.publish(this->gripper_suction_sys_control);

                    this->gripper_closed = false;
                    res.success.data = true;
                    return true;
                }
                else
                {
                    displayPlannerError(execution_result);
                    res.success.data = false;
                    return true;
                }
            }
            else
            {
                this->is_drop_plan = false; // for next pick or single goal planning
                cout << "Failed to find plan" << endl;
                res.success.data = false;
                return true;
            }
        }
        else
        {
            this->is_drop_plan = false; // for next pick or single goal planning
            cout << "Failed to find plan" << endl;
            res.success.data = false;
            return true;
        }
    }
    else
    {
        this->is_drop_plan = false; // for next pick or single goal planning
        cout << "Failed to find plan" << endl;
        res.success.data = false;
        return true;
    }
}

geometry_msgs::Point RobotArm::alignAxes(vector<tf::Vector3> &valley_detect_axes, vector<tf::Vector3> object_axes,
                                         tf::Vector3 valley_centroid, tf::Vector3 obj_hold_pt, tf::Vector3 obj_model_centroid, bool invert_axes)
{
    // form a vector from object model centroid to point on object where it is held
    tf::Vector3 vector_pc(obj_hold_pt);
    vector_pc -= obj_model_centroid;

    // find projections to get the distance at which object is held from its model centroid
    double proj_major_axis = vector_pc.dot(object_axes[0]);
    double proj_minor_axis = vector_pc.dot(object_axes[1]);

    // form the normal for object and box opening
    tf::Vector3 normal_valley = valley_detect_axes[0].cross(valley_detect_axes[1]);
    tf::Vector3 normal_object = object_axes[0].cross(object_axes[1]);

    // check whether the orientation of the valley has to be changed
    double direction_check = normal_valley.dot(normal_object);

    if(invert_axes)// axes has to be inverted once the valley axes are aligned with the model axes
    {
        valley_detect_axes[0] *= -1;
        valley_detect_axes[1] *= -1;
    }
    else
    {
        // In case normals are not in same direction the invert the major axis of valley detection
        if(direction_check < 0)
        {
            // just inverting anyone axis will work
            valley_detect_axes[0] *= -1;
        }
    }

    tf::Vector3 pt_valley_mj_axis = getEndpt(valley_detect_axes[0], valley_centroid, proj_major_axis);
    tf::Vector3 hold_pt_valley = getEndpt(valley_detect_axes[1], pt_valley_mj_axis, proj_minor_axis);

    geometry_msgs::Point hold_pt;
    hold_pt.x = hold_pt_valley.getX();
    hold_pt.y = hold_pt_valley.getY();
    hold_pt.z = hold_pt_valley.getZ();

    return hold_pt;
}
