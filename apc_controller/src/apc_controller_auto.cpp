#include <iostream>
#include <apc_controller.h>


void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    //    sleep(1);
    //    exit(signum);
    exit(0);
}

//double height[4] = {0.5, 0.5, -0.15, -0.15};
using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc,argv,"apc_controller_auto");

    ros::NodeHandle nh;

    ros::ServiceClient linesRackDetectService = nh.serviceClient<lines_rack_det::DetectShelf>("/lines_rack_detect");

    ros::ServiceServer shelf_dataService = nh.advertiseService("/shelf_data", shelfDataCallback);

    ros::Publisher simJointPublisher = nh.advertise<sensor_msgs::JointState>("/apc_controller/joint_states",1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
    bool get_cloud = true;
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
                                                                       boost::bind(kinectPCCallback, _1, boost::ref(cloud), boost::ref(get_cloud)));


    vector<double> wam_current_jts(7);
    vector<double> wam_desired_jts(7);

    ros::Subscriber sub_wam_joints = nh.subscribe<sensor_msgs::JointState>
            ("/wam/joint_states",1,boost::bind(wamJointStateCallback,_1,boost::ref(wam_current_jts)));


    ros::ServiceClient poseService = nh.serviceClient<apc_controller::JointMove>("/wam/joint_move");
    apc_controller::JointMove joint_srv_start,joint_srv_home, joint_srv_start_int;
    apc_controller::JointMove joint_srv_toteC1, joint_srv_toteC2, joint_srv_toteC3;


    ros::Publisher vaccumDataPub = nh.advertise<std_msgs::Int16>("/toggle_led",1);
    ros::Publisher vaccumCleanerPub = nh.advertise<std_msgs::Int16>("/vacuum_topic",1);
    int ir_val, metal_val;
    bool ir_data_available = false, metal_data_available = false;
    ros::Subscriber ir_sub = nh.subscribe<std_msgs::Int16>("/ir_topic", 1, boost::bind(irSensorCallback, _1, boost::ref(ir_val), boost::ref(ir_data_available)));
    ros::Subscriber metal_sub = nh.subscribe<std_msgs::Int16>("/metal_topic", 1, boost::bind(metalSensorCallback, _1, boost::ref(metal_val), boost::ref(metal_data_available)));

    ros::ServiceClient pick_obj_service = nh.serviceClient<json_maker::get_bin_object>("/pick_object_service");
    ros::ServiceClient pick_status_write_service = nh.serviceClient<json_maker::write_pick_status>("/pick_object_status_service");

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/bin_corner",10);


    ifstream startPosfile;
    std::string dataPath_start = ros::package::getPath("apc_controller").append("/data/start_pos.txt");

    startPosfile.open(dataPath_start.c_str());
    if(!startPosfile.is_open())
    {
        cerr << "Error opening file start pos" << endl;
        exit(0);
    }

    ifstream startIntPosfile;
    std::string dataPath_startInt = ros::package::getPath("apc_controller").append("/data/start_intermediate.txt");
    startIntPosfile.open(dataPath_startInt.c_str());
    if(!startIntPosfile.is_open())
    {
        cerr << "Error opening file intermediate start pos" << endl;
        exit(0);
    }

    ifstream homePosFile;
    std::string dataPath_home = ros::package::getPath("apc_controller").append("/data/home_pos.txt");

    homePosFile.open(dataPath_home.c_str());
    if(!homePosFile.is_open())
    {
        cerr << "Error opening gile" << endl;
        exit(0);
    }

    ifstream toteC1File;
    std::string path_tote = ros::package::getPath("apc_controller").append("/data/toteC1_pos.txt");

    toteC1File.open(path_tote.c_str());
    if(!toteC1File.is_open())
    {
        cerr << "Error opening toteC1_position file" << endl;
        exit(0);
    }

    ifstream toteC2File;
    path_tote = ros::package::getPath("apc_controller").append("/data/toteC2_pos.txt");

    toteC2File.open(path_tote.c_str());
    if(!toteC2File.is_open())
    {
        cerr << "Error opening toteC2_position file" << endl;
        exit(0);
    }

    ifstream toteC3File;
    path_tote = ros::package::getPath("apc_controller").append("/data/toteC1_pos.txt");

    toteC3File.open(path_tote.c_str());
    if(!toteC3File.is_open())
    {
        cerr << "Error opening toteC3_position file" << endl;
        exit(0);
    }

    //-------------Real robot movement data


    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        startPosfile >> temp;
        joint_srv_start.request.joints.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        startIntPosfile >> temp;
        joint_srv_start_int.request.joints.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        homePosFile >> temp;
        joint_srv_home.request.joints.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;;
        toteC1File >> temp1;
        toteC2File >> temp2;
        toteC3File >> temp3;
        joint_srv_toteC1.request.joints.push_back(temp1);
        joint_srv_toteC2.request.joints.push_back(temp2);
        joint_srv_toteC3.request.joints.push_back(temp3);
    }



    //---------------------- Simulation data


    vector<double> home_pose;
    vector<double> start_pose;
    vector<double> start_int_pose;

    vector<double> toteC1_pose, toteC2_pose, toteC3_pose;

    startPosfile.seekg(0,std::ios_base::beg);
    startIntPosfile.seekg(0,std::ios_base::beg);
    homePosFile.seekg(0,std::ios_base::beg);
    toteC1File.seekg(0,std::ios_base::beg);
    toteC2File.seekg(0,std::ios_base::beg);
    toteC3File.seekg(0,std::ios_base::beg);

    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        startPosfile >> temp;
        start_pose.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        startIntPosfile >> temp;
        start_int_pose.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp = 0.0;
        homePosFile >> temp;
        home_pose.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;
        toteC1File >> temp1;
        toteC2File >> temp2;
        toteC3File >> temp3;
        toteC1_pose.push_back(temp1);
        toteC2_pose.push_back(temp2);
        toteC3_pose.push_back(temp3);
    }


    // Shelf detection variables
    lines_rack_det::DetectShelf dhselfCentroid;
    vector<double> centroidArray;
    int shelfBinIndex = 0;


#if(XWAM_MOTION)
    ros::ServiceClient motionFwdBckService = nh.serviceClient<xwamotion_laser::FwdBck>("/motion/fwdbck");
    ros::ServiceClient motionUpDownService = nh.serviceClient<xwamotion_laser::UpDown>("/motion/updown");

    xwamotion_laser::FwdBck xwam_fwdback;
    xwamotion_laser::UpDown xwam_updown;
    double xwam_dist_err, xwam_height_err;
#endif

#if(TRAJECTORY_RPT)
    ros::ServiceClient trajRptService = nh.serviceClient<trajectory_rpt::TrajRpt>("/trajectory/rpt");
#endif

#if(RCNN)
    ros::ServiceClient objDetectRCNNService = nh.serviceClient<apc_controller::objectDetect>("/detect_object");
#endif

//    ros::ServiceClient binCropService = nh.serviceClient<apc_controller::CropBin>("/crop/bin");

#if(RACK_REGISTRATION)
    // rack registration service to register rack as per ashish
    ros::ServiceClient rackRegistrationService = nh.serviceClient<apc_controller::rack_registration>("/rack_registration");
#endif

    double bin_angles[][7] = {{1.523,  0.053,  0.108,  1.757, -0.542,  1.379,  0.264},
                              {1.523,  0.053,  0.108,  1.757, -0.542,  1.379,  0.264},
                              {1.523,  0.053,  0.108,  1.757, -0.542,  1.379,  0.264},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590},
                              {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590}
                             };

//    double bin_angles[][7] = {{1.450, -0.128,  0.171,  2.022, -0.463,  1.082, -0.552},
//                              {1.450, -0.128,  0.171,  2.022, -0.463,  1.082, -0.552},
//                              {1.450, -0.128,  0.171,  2.022, -0.463,  1.082, -0.552},
//                              {1.529, -0.178,  0.086,  2.126, -0.427,  0.893, -1.06},
//                              {1.529, -0.178,  0.086,  2.126, -0.427,  0.893, -1.06},
//                              {1.529, -0.178,  0.086,  2.126, -0.427,  0.893, -1.06},
//                              {1.517, -0.408,  0.088,  2.363, -0.253,  0.451, -1.371},
//                              {1.517, -0.408,  0.088,  2.363, -0.253,  0.451, -1.371},
//                              {1.517, -0.408,  0.088,  2.363, -0.253,  0.451, -1.371},
//                              {1.481, -0.578,  0.123,  2.552, -0.230,  0.305, -1.320},
//                              {1.481, -0.578,  0.123,  2.552, -0.230,  0.305, -1.320},
//                              {1.481, -0.578,  0.123,  2.552, -0.230,  0.305, -1.320}
//                             };

    double left_view_angles[][7] = {{0.708,  0.223,  1.271,  1.774, -0.094,  0.943,  0.025},
                                    {0.708,  0.223,  1.271,  1.774, -0.094,  0.943,  0.025},
                                    {0.708,  0.223,  1.271,  1.774, -0.094,  0.943,  0.025},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                    {0.780,  0.308,  1.323,  1.947,  0.070,  0.590,  0.155},
                                   };
    double right_view_angles[][7] = {{2.470,  0.266, -1.443,  1.680, -0.185,  1.148,  0.285},
                                     {2.470,  0.266, -1.443,  1.680, -0.185,  1.148,  0.285},
                                     {2.470,  0.266, -1.443,  1.680, -0.185,  1.148,  0.285},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                     {2.138,  0.251, -0.977,  2.050, -0.045,  0.657, -0.300},
                                    };

    double drop_l1[][7] = {{-1.538,  1.003,  0.006, 2.149, 0.046, -0.471,  0.029},
                           {-1.368,  1.016,  0.038, 2.149, 0.036, -0.466,  0.076},
                           {-1.684,  1.002, -0.004, 2.149, 0.123, -0.426, -0.030}
                          };
    double drop_l2[][7] = {{-1.538, 1.003,  0.006, 2.149, 0.046, -0.471,  0.029},
                           {-1.368, 1.016,  0.038, 2.149, 0.036, -0.466,  0.076},
                           {-1.684, 1.002, -0.004, 2.149, 0.123, -0.426, -0.030}
                          };
    double drop_l3[][7] = {{-1.588, 0.746, 0.144, 2.534,  0.218, -0.868, -0.128},
                           {-1.501, 0.814, 0.195, 2.476,  0.051, -0.846,  0.018},
                           {-1.866, 0.814, 0.193, 2.477, -0.005, -0.817,  0.052}
                          };
    double drop_l4[][7] = {{-1.638, 0.520, 0.056, 2.686, -0.010, -0.932, -0.024},
                           {-1.343, 0.514, 0.081, 2.685,  0.113, -0.997, -0.032},
                           {-1.791, 0.496, 0.094, 2.599,  0.007, -0.821,  0.032}
                          };
    double drop_itd[][7] = {{ 1.672, -0.682, -0.031, 2.682, -0.111, -0.066,  0.009},
                            { 0.035,  0.009,  0.007, 2.300,  0.140, -0.282, -0.303},
                            {-1.657,  0.252,  0.334, 2.392,  0.315, -0.348, -0.155}
                           };

    double drop_front_r12[][7] = {{1.257, 0.895, -0.052, 2.252, 0.206, 0.0, 0.0},
                                  {1.504, 0.892, -0.042, 2.252, 0.167, 0.0, 0.0},
                                  {1.816, 0.884, -0.046, 2.285, 0.205, 0.0, 0.0}};

    double drop_front_r3[][7] = {{1.669, 0.445, 0.078, 2.524, 0.193, 0.0, 0.0},
                                 {1.483, 0.491, -0.019, 2.472, 0.163, 0.0, 0.0},
                                 {1.201, 0.574, 0.001, 2.270, 0.121, 0.0, 0.0}};

    double drop_front_r4[][7] = {{1.483, 0.196, -0.298, 2.456, -0.052, 0.0, 0.0},
                                 {1.602, 0.147, -0.164, 2.458, -0.057, 0.0, 0.0},
                                 {1.733, 0.149, 0.029, 2.475, -0.055, 0.0, 0.0}};

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

    int ir_exception_list[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
    bool service_status_flag = false;

    double error_height = 0.0;
    double error_motion = 0.0;

    int bin_iteration = 0;


    while(ros::ok()  && bin_iteration < 12)
    {

        cout << "Bin iteration \t" << bin_iteration << endl;

        service_status_flag = true;
        int object_to_be_picked = 0;
        int bin_number = 0;

        //--------------- Getting object id and bin number from json File------------------------------

        json_maker::get_bin_object get_json_obj_bin_id;


        if(pick_obj_service.call(get_json_obj_bin_id))
        {
            service_status_flag = true;
            object_to_be_picked = get_json_obj_bin_id.response.obj_id.data;
            bin_number = get_json_obj_bin_id.response.bin_num.data;
            cout << "Object data extracted from JSON file successfully" << endl;
        }
        else
        {
            service_status_flag = false;
            cout << "Object data either not extracted or not present in json service" << endl;
        }





        cout << "Object id \t " << object_to_be_picked << endl;
        cout << "Bin id \t" << bin_number << endl;


//        bin_iteration++;
//        continue;

        //-------------------------------- Robot reaching at start position ---------------------------------------

        if(service_status_flag && bin_iteration == 0)
        {
            service_status_flag = false;


            cout << "Start position call" << endl;


            publishJointStateInSimulation(start_int_pose,simJointPublisher);

            if(poseService.call(joint_srv_start_int))
            {
                cout << "Start intermediate position reached " << endl;

                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start_int.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                {
                    cout << "Start intermediate position reached " << endl;
                }
                else
                    cout << "Failed to reach desired wam position" << endl;

            }
            else
            {
                ROS_ERROR("Failed to call service");
            }


            publishJointStateInSimulation(start_pose,simJointPublisher);
            if(poseService.call(joint_srv_start))
            {

                cout << "Start position reached " << endl;

                service_status_flag = true;

                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                {
                    sleep(2);
                    cout << "Start position reached " << endl;
                }
                else
                    cout << "Failed to reach desired wam position" << endl;

            }
            else
            {

                service_status_flag = false;
                ROS_ERROR("Failed to call service");
            }

        }






        double robot_motion_each_iteration[12] = {0.0, 0.0,0.0, 0.0, 0.27, 0.0, 0.0, 0.0, -0.27, 0.0, 0.0, 0.0};



        // ---------------------------------- Moving robot forward and backward in the workspace ----------------

        if(service_status_flag )
        {

            service_status_flag = false;
            // for moving robot fwd 0.3m
            double d;

            d = robot_motion_each_iteration[bin_iteration] + error_motion;

            xwam_fwdback.request.distance.data = d;
            if(motionFwdBckService.call(xwam_fwdback))
            {
                cout << "Successful forward motion" << endl;
                xwam_dist_err = xwam_fwdback.response.error.data;
                d -= xwam_dist_err;
                error_motion -= xwam_dist_err;
                cout << "Robot moved in x: " << d << endl;
                translateCentroidCorners(d,0.0,0.0);
                service_status_flag = true;
            }
            else
            {
                cout << "Failed to call xwam forward motion service" << endl;
                service_status_flag = false;
            }


        }


        // ----------------------------------------------------------------------------------------SPECIAL_CASE-----------------------------------------
        //--------------------------------------------------------------------------------------------------------------------
        if(bin_iteration == 8 && service_status_flag)
        {


            cout << "Special case movement for rack detection\t" << endl;

            // Start position call
            if(service_status_flag)
            {
                service_status_flag = false;

                cout << "Start position call" << endl;
                publishJointStateInSimulation(start_pose,simJointPublisher);
                if(poseService.call(joint_srv_start))
                {
                    cout << "Start position reached " << endl;

                    service_status_flag = true;
                    for(int i = 0; i < wam_desired_jts.size(); i++)
                        wam_desired_jts[i] = joint_srv_start.request.joints[i];
                    if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    {
                        sleep(4);
                        cout << "Start position reached " << endl;
                    }
                    else
                        cout << "Failed to reach desired wam position" << endl;
                }
                else
                {
                    service_status_flag = false;
                    ROS_ERROR("Failed to call service");
                }

            }

            // ----------------------------------- Robot doing rack detection using kinect---------------------------------


            if(service_status_flag)
            {

                service_status_flag = false;


                cout << "Lines Rack detection call" << endl;
                if(linesRackDetectService.call(dhselfCentroid))
                {
                    centroidArray.clear();
                    for(std::vector<double>::const_iterator it = dhselfCentroid.response.centroids.data.begin();
                        it != dhselfCentroid.response.centroids.data.end();++it)
                    {
                        centroidArray.push_back(*it);
                    }

                    int b = 0;
                    for(std::vector<double>::const_iterator it = dhselfCentroid.response.bin_corners.data.begin();
                        it < dhselfCentroid.response.bin_corners.data.end(); it+=3)
                    {
                        double x = *it;
                        double y = *(it+1);
                        double z = *(it+2);
                        tf::Vector3 v(x,y,z);
                        bin_corners[b] = v;
                        b++;
                    }
//                    for(int i = 0; i<bin_corners.size(); i+=4)
//                    {
//                        cout << "Corners of Bin: " << i/4 << endl;
//                        for(int j = 0; j<4; j++)
//                            cout << "[" << bin_corners[i+j].getX() << "," << bin_corners[i+j].getY() << "," << bin_corners[i+j].getZ() << "]" << endl;
//                        cout << endl;
//                    }
//                    cout << "Copied Bin Corners" << endl;

                    int count = 0;
                    if(centroidArray.size() > 0)
                    {
//                        cout << "Bin centroids: \n";
                        for(int i=0;i<int(NO_OF_BINS);i++)
                        {
                            centroidVector[i][0] = centroidArray[count];
                            centroidVector[i][1] = centroidArray[count+1];
                            centroidVector[i][2] = centroidArray[count+2];
                            count += 3;
//                            cout << "Centroid of Bin: " << i << endl;
//                            cout << "[" << centroidVector[i][0] << " " << centroidVector[i][1] << " " << centroidVector[i][2] << "]" << endl;
                        }
//                        cout << "Copied Bin Centroids" << endl;
                    }
                    cout << "lines rack detect Service done " << endl;

                    got_shelf_data = true;
                    service_status_flag = true;
                }
                else
                {
                    service_status_flag = false;
                    cout << "lines rack detect service not done" << endl;
                }
            }

            //------------------------------------------------------------- Calling bin resgitration service --------------------------

            if(service_status_flag)
            {
                service_status_flag = false;

                get_cloud = true;
                while(get_cloud && ros::ok())
                {
                    get_cloud = true;
                    ros::spinOnce();
                }
                cout << "Cloud found\nCaution: Arm should be in start position" << endl;
                // code to send the corners points between 3 horizontal lines
                // send bin_corners indexed 12(left bottom), 21(left top), 26(right top), 35(right bottom). This is ordered according to requirement by Ashish
                int idx[4] = {26,12,21,35};
                apc_controller::rack_registration rack_reg;
                tf::StampedTransform WAMtoKttransform;
                getWAMBaseToKinect(WAMtoKttransform);
                for(int j=0; j<4; j++)
                {
                    tf::Vector3 corner = bin_corners[idx[j]];
                    tf::Vector3 tf_corner = WAMtoKttransform*corner;
                    rack_reg.request.corner_points.data.push_back(tf_corner.getX());
                    rack_reg.request.corner_points.data.push_back(tf_corner.getY());
                    rack_reg.request.corner_points.data.push_back(tf_corner.getZ());
                }

                pcl::toROSMsg(*cloud,rack_reg.request.input_rgb_cloud);
                //            rack_reg.request.input_rgb_cloud = *pc_msg;
                cout << "Called rack registration service" << endl;
                if(rackRegistrationService.call(rack_reg))
                {
                    cout << "Success to send bin corner data for rack registration" << endl;
                    service_status_flag = true;
                }
                else
                {
                    cout << "Failed to call service for rack registration" << endl;
                    //                success = false;
                    service_status_flag = false;
                }
            }

            if(service_status_flag)
            {

                service_status_flag = false;
                // for moving robot fwd 0.3m
                double d;
                d = robot_motion_each_iteration[bin_iteration] + error_motion;

                xwam_fwdback.request.distance.data = d;
                if(motionFwdBckService.call(xwam_fwdback))
                {
                    cout << "Successful forward motion" << endl;
                    xwam_dist_err = xwam_fwdback.response.error.data;
                    d -= xwam_dist_err;
                    error_motion -= xwam_dist_err;
                    cout << "Robot moved in x: " << d << endl;
                    translateCentroidCorners(d,0.0,0.0);
                    service_status_flag = true;
                }
                else
                {
                    cout << "Failed to call xwam forward motion service" << endl;
                    service_status_flag = false;
                }
            }

        }





        cout << "Changing height of the robot \t" << bin_iteration  << endl;



        //------------------------------------------------ Chaning height of the robot according to the bin ---------------------------





        //        double robot_heights_each_iteration[12] = {0.0, 0.0,-0.15, -0.15, 0.50, 0.0, -0.15, -0.15, 0.50, 0.0, -0.15, -0.15};

//        double robot_heights_each_iteration[12] = {0.0, 0.0,-0.16, -0.19, 0.0, 0.19, 0.30, 0.0, 0.0, 0.0, -0.16, -0.19};
        double robot_heights_each_iteration[12] = {0.0, 0.0,-0.16, -0.00, 0.0, 0.00, 0.30, 0.0, 0.0, 0.0, -0.16, -0.00};
//        double robot_heights_each_iteration[12] = {0.0, -0.11,-0.15, -0.06, 0.0, 0.06, 0.15, 0.11, 0.0, -0.11, -0.15, -0.06};


        if(service_status_flag)
        {

            service_status_flag = false;
            // for moving robot up 0.2m
            double h = 0.0;

            if(bin_iteration == 7)
                error_height = 0;

            h = robot_heights_each_iteration[bin_iteration]-error_height;

            cout << "Bin iter \t" << bin_iteration << endl;
            cout << "Error height \t" << error_height << endl;
            cout << "Height to move  \t" << h << endl;

            xwam_updown.request.height.data = h;
            if(motionUpDownService.call(xwam_updown))
            {
                service_status_flag = true;
                cout << "Successful upward motion" << endl;
                h += xwam_updown.response.error.data;
                error_height += xwam_updown.response.error.data;
                //                current_height = h;
                cout << "Robot moved in z: " << h << endl;

                translateCentroidCorners(0.0,0.0,h);
                //                bin_iteration++;
            }
            else
            {
                cout << "Failed to call xwam upward motion service" << endl;
                service_status_flag = false;
            }
        }








        // ----------------------------------- Robot doing rack detection using kinect---------------------------------


        if(service_status_flag && bin_iteration == 0)
        {

            service_status_flag = false;


            cout << "Lines Rack detection call" << endl;
            if(linesRackDetectService.call(dhselfCentroid))
            {
                centroidArray.clear();
                for(std::vector<double>::const_iterator it = dhselfCentroid.response.centroids.data.begin();
                    it != dhselfCentroid.response.centroids.data.end();++it)
                {
                    centroidArray.push_back(*it);
                }

                int b = 0;
                for(std::vector<double>::const_iterator it = dhselfCentroid.response.bin_corners.data.begin();
                    it < dhselfCentroid.response.bin_corners.data.end(); it+=3)
                {
                    double x = *it;
                    double y = *(it+1);
                    double z = *(it+2);
                    tf::Vector3 v(x,y,z);
                    bin_corners[b] = v;
                    b++;
                }
//                for(int i = 0; i<bin_corners.size(); i+=4)
//                {
//                    cout << "Corners of Bin: " << i/4 << endl;
//                    for(int j = 0; j<4; j++)
//                        cout << "[" << bin_corners[i+j].getX() << "," << bin_corners[i+j].getY() << "," << bin_corners[i+j].getZ() << "]" << endl;
//                    cout << endl;
//                }
//                cout << "Copied Bin Corners" << endl;

                int count = 0;
                if(centroidArray.size() > 0)
                {
                    //                    cout << "Bin centroids: \n";
                    for(int i=0;i<int(NO_OF_BINS);i++)
                    {
                        centroidVector[i][0] = centroidArray[count];
                        centroidVector[i][1] = centroidArray[count+1];
                        centroidVector[i][2] = centroidArray[count+2];
                        count += 3;
//                        cout << "Centroid of Bin: " << i << endl;
//                        cout << "[" << centroidVector[i][0] << " " << centroidVector[i][1] << " " << centroidVector[i][2] << "]" << endl;
                    }
//                    cout << "Copied Bin Centroids" << endl;
                }
                cout << "lines rack detect Service done " << endl;

                got_shelf_data = true;

                service_status_flag = true;

            }
            else
            {
                service_status_flag = false;
                cout << "lines rack detect service not done" << endl;
            }


        }


        // Chwcking blacklisted objects
//        int blacklist_obj[] = {26, 27, 30, 32, 35, 39}, num_blacklist = 6;
//        bool found_blacklist_obj = false;
//        for(int i=0; i<num_blacklist; i++)
//        {
//            if(object_to_be_picked == blacklist_obj[i])
//            {
//                bin_iteration++;
//                service_status_flag = false;
//                found_blacklist_obj = true;
//                break;
//            }
//        }
//        if(found_blacklist_obj)
//            continue;

        int blacklist_bin[] = {9, 10, 11}, num_blacklist_bin = 3;
        bool found_blacklist_bin = false;
        for(int i=0; i<num_blacklist_bin; i++)
        {
            if(bin_number == blacklist_bin[i])
            {
                bin_iteration++;
                service_status_flag = false;
                found_blacklist_bin = true;
                break;
            }
        }
        if(found_blacklist_bin)
            continue;





        //------------------------------------------------------------- Calling bin resgitration service --------------------------

        if(service_status_flag && bin_iteration == 0)
        {
            service_status_flag = false;


            get_cloud = true;
            while(get_cloud && ros::ok())
            {
                get_cloud = true;
                ros::spinOnce();
            }
            cout << "Cloud found\nCaution: Arm should be in start position" << endl;
            // code to send the corners points between 3 horizontal lines
            // send bin_corners indexed 12(left bottom), 21(left top), 26(right top), 35(right bottom). This is ordered according to requirement by Ashish
            int idx[4] = {26,12,21,35};
            apc_controller::rack_registration rack_reg;
            tf::StampedTransform WAMtoKttransform;
            getWAMBaseToKinect(WAMtoKttransform);
            for(int j=0; j<4; j++)
            {
                tf::Vector3 corner = bin_corners[idx[j]];
                tf::Vector3 tf_corner = WAMtoKttransform*corner;
                rack_reg.request.corner_points.data.push_back(tf_corner.getX());
                rack_reg.request.corner_points.data.push_back(tf_corner.getY());
                rack_reg.request.corner_points.data.push_back(tf_corner.getZ());
            }

            pcl::toROSMsg(*cloud,rack_reg.request.input_rgb_cloud);
            //            rack_reg.request.input_rgb_cloud = *pc_msg;
            cout << "Called rack registration service" << endl;
            if(rackRegistrationService.call(rack_reg))
            {
                cout << "Success to send bin corner data for rack registration" << endl;
                service_status_flag = true;
            }
            else
            {
                cout << "Failed to call service for rack registration" << endl;
                //                success = false;
                service_status_flag = false;
            }
        }






        //------------------------------- Reach bin for viewing objects using kinect -----------------------------

         cout << "______________________________" << endl;
         cout << "No of attempts 1" << endl;
         cout << "------------------------------- " << endl;

        int bin_idx = bin_number;
        apc_controller::JointMove jt_move;

//        Make second attempt at picking the object in case of no picking in first attempt
//        for(int attempt = 0; attempt < 1; attempt++)
        {
            if(service_status_flag)
            {

                service_status_flag = false;

                jt_move.request.joints.clear();

                for(int j=0; j<7; j++)
                    jt_move.request.joints.push_back(bin_angles[bin_idx][j]);

                vector<double> binPose(7);
                for(int j=0;j<7;j++)
                    binPose[j] = jt_move.request.joints[j];
                publishJointStateInSimulation(binPose,simJointPublisher);

                if(poseService.call(jt_move))// call the service
                {
                    for(int i = 0; i < wam_desired_jts.size(); i++)
                        wam_desired_jts[i] = jt_move.request.joints[i];
                    if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    {
                        sleep(4);
                        service_status_flag = true;
                        cout << "service done: arm reached bin view " << endl;
                    }
                    else
                    {
                        cout << "Failed to reach desired wam position" << endl;
                        service_status_flag = false;
                    }
                }
                else
                {
                    cout << "service not done: arm reached bin view " << endl;
                    service_status_flag = false;
                }


            }




            //----------------------------------- Call image processing algorithm to get centroid of object ------------------------------

            geometry_msgs::Point obj_centroid_rpt, obj_normal_rpt, obj_left, obj_top, obj_right;


            // *********************************
            // Display bin corners after transformation
            //        {
            //            tf::StampedTransform WAMtoKttransform;
            //            getWAMBaseToKinect(WAMtoKttransform);
            //            vector<tf::Vector3> tf_corners(bin_corners.size());
            //            for(int j=0; j<bin_corners.size(); j++)
            //            {
            //                tf_corners[j] = WAMtoKttransform*bin_corners[j];
            //            }

            //            sphereMarker(tf_corners, marker_array_pub);
            //            cout << "Published corners markers" << endl;
            //        }

            int obj_id_rpt;
            //        if(0)
            if(service_status_flag)
            {

                service_status_flag = false;
                obj_id_rpt = object_to_be_picked;

                //            break;


                get_cloud = true;
                while(get_cloud && ros::ok())
                {
                    get_cloud = true;
                    ros::spinOnce();
                }
                cout << "Cloud found" << endl;


                apc_controller::objectDetect obj_detect_rcnn;
                obj_detect_rcnn.request.bin_num.data = bin_idx;
                obj_detect_rcnn.request.bin_object_list.data.push_back(obj_id_rpt);
                pcl::toROSMsg(*cloud,obj_detect_rcnn.request.input_rgb_cloud);
                //                    obj_detect_rcnn.request.input_rgb_cloud = *pc_msg;
                sleep(1);
                tf::StampedTransform WAMtoKttransform;
                getWAMBaseToKinect(WAMtoKttransform);
#if(SEND_END_CORNERS)
                // code to send the corners points between 3 horizontal lines
                // send bin_corners indexed 12(left bottom), 21(left top), 26(right top), 35(right bottom). This is ordered according to requirement by Ashish
                int idx[4] = {26,12,21,35};
                for(int j=0; j<4; j++)
                {
                    tf::Vector3 end_corner = bin_corners[idx[j]];
                    tf::Vector3 tfend_corner = WAMtoKttransform*end_corner;
                    // copy end corners
                    obj_detect_rcnn.request.transformation_old_new.data.push_back(tfend_corner.getX());
                    obj_detect_rcnn.request.transformation_old_new.data.push_back(tfend_corner.getY());
                    obj_detect_rcnn.request.transformation_old_new.data.push_back(tfend_corner.getZ());
                }
#endif


                cout << "Called RCNN object detection service" << endl;
                cout << "Bin: " << bin_idx << ", Obj id sent: " << obj_id_rpt << endl;
                cout << "Object to pick: " << model_names[obj_id_rpt-1] << endl;
                if(objDetectRCNNService.call(obj_detect_rcnn))//call object detection service method 1
                {
                    cout << "service done: object detection obtained by method "  << endl;
                    cout << "Obj centroid: [" << obj_detect_rcnn.response.centroid.x << " " << obj_detect_rcnn.response.centroid.y << " " << obj_detect_rcnn.response.centroid.z << "]" << endl;
                    cout << "Obj left: [" << obj_detect_rcnn.response.obj_left.x << " " << obj_detect_rcnn.response.obj_left.y << " " << obj_detect_rcnn.response.obj_left.z << "]" << endl;
                    cout << "Obj top: [" << obj_detect_rcnn.response.obj_top.x << " " << obj_detect_rcnn.response.obj_top.y << " " << obj_detect_rcnn.response.obj_top.z << "]" << endl;
                    cout << "Obj right: [" << obj_detect_rcnn.response.obj_right.x << " " << obj_detect_rcnn.response.obj_right.y << " " << obj_detect_rcnn.response.obj_right.z << "]" << endl;
                    // copy object centroid and object normal to local variable
                    kinectToWAMBase(obj_detect_rcnn.response.centroid, obj_centroid_rpt);
                    kinectToWAMBase(obj_detect_rcnn.response.obj_left, obj_left);
                    kinectToWAMBase(obj_detect_rcnn.response.obj_top, obj_top);
                    kinectToWAMBase(obj_detect_rcnn.response.obj_right, obj_right);
                    kinectToWAMVector(obj_detect_rcnn.response.normal, obj_normal_rpt);

                    service_status_flag = true;
                }
                else
                {
                    cout << "service not done: object detection obtained by method "  << endl;
                    service_status_flag = false;
                }


            }


            //        service_status_flag = true;// remove this when if(0) is removed
            // ------------------------------- Call planning algorithm to pick the object and put it into tote -------------------------

            //        if(0)
            if(service_status_flag)
            {
                bool success = true;

                service_status_flag = false;

                trajectory_rpt::TrajRpt trajRpt;
                // Call Trajectory planning service
                if(success)
                {
                    // copy the request variables to call trajectory generation service for RPT
                    for(int i=0; i<centroidVector[bin_idx].size(); i++)
                        trajRpt.request.bin_centroid.data.push_back(centroidVector[bin_idx][i]);
                    cout << "Bin centroid: " << centroidVector[bin_idx][0] << " " << centroidVector[bin_idx][1] << " " << centroidVector[bin_idx][2] << endl;
                    for(int i=0; i<4; i++)
                    {
                        tf::Vector3 corner = bin_corners[bin_idx*4 + i];
                        trajRpt.request.bin_corners.data.push_back(corner.getX());
                        trajRpt.request.bin_corners.data.push_back(corner.getY());
                        trajRpt.request.bin_corners.data.push_back(corner.getZ());
                    }
                    trajRpt.request.bin_num.data = bin_idx;
                    trajRpt.request.obj_centroid = obj_centroid_rpt;
                    trajRpt.request.obj_normal = obj_normal_rpt;
                    trajRpt.request.obj_left = obj_left;
                    trajRpt.request.obj_top = obj_top;
                    trajRpt.request.obj_right = obj_right;
                    trajRpt.request.obj_id.data = obj_id_rpt;

                    cout << "\n Object centroid: [" << trajRpt.request.obj_centroid.x << " " << trajRpt.request.obj_centroid.y << " " << trajRpt.request.obj_centroid.z << "]\n";
                    // call service to find trajectory to reach the object
                    if(trajRptService.call(trajRpt))
                    {
                        cout << "service done: RPT Trajectory planning done" << endl;
                        success = true;
                    }
                    else
                    {
                        cout << "service not done: RPT Trajectory planning done" << endl;
                        success = false;
                    }
                }

                // call service to send trajectory angles to the robot
                if(success)
                {
                    // copy the trajectory joint angle values into the local variables
                    int n_jts = 7;// Number of joints on arm
                    int n_trj = trajRpt.response.jts_reach.data.size()/n_jts;
                    vector< vector<double> > traj_reach_obj = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

                    n_trj = trajRpt.response.jts_back.data.size()/n_jts;
                    vector< vector<double> > traj_come_out = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

                    cout << "Copy Trajectory forward angles: " << traj_reach_obj.size() << "\n[";
                    int r = 0,s=0;
                    for(vector<double>::const_iterator it = trajRpt.response.jts_reach.data.begin(); it!=trajRpt.response.jts_reach.data.end(); ++it)
                    {
                        traj_reach_obj[r][s] = *it;
                        cout << traj_reach_obj[r][s] << " ";
                        s++;
                        if(s==n_jts)
                        {
                            cout << "]\n" << ": [";
                            r++;
                            s=0;
                        }
                    }
                    cout << "Copy Trajectory Reverse angles: " << traj_come_out.size() << "\n ";
                    r = 0,s=0;
                    for(vector<double>::const_iterator it = trajRpt.response.jts_back.data.begin(); it!=trajRpt.response.jts_back.data.end(); ++it)
                    {
                        traj_come_out[r][s] = *it;
                        cout << traj_come_out[r][s] << " ";
                        s++;
                        if(s==n_jts)
                        {
                            cout << "\n";
                            r++;
                            s=0;
                        }
                    }
                    cout << endl;
                    for(int i=0; i<traj_reach_obj.size(); i++)
                    {
                        if(success)
                        {
                            jt_move.request.joints.clear();
                            for(int j=0; j<n_jts; j++)
                                jt_move.request.joints.push_back(traj_reach_obj[i][j]);

                            if(trajRpt.response.idx_bend.data.size()>0)
                            {
                                if(i==trajRpt.response.idx_bend.data[0])
                                {
                                    // call service to bend the pipe at eef
                                    if(1)
                                    {
                                        cout << "service done: Tool eef bent" << endl;
                                        success = true;
                                    }
                                    else
                                    {
                                        cout << "service not done: Tool eef not bent" << endl;
                                        success = false;
                                    }
                                }
                            }

#if(ROBOT_ANGLE_SIMULATION)
                            vector<double> binPose(7);
                            for(int j=0;j<7;j++)
                                binPose[j] = jt_move.request.joints[j];
                            publishJointStateInSimulation(binPose,simJointPublisher);
#endif

                            if(poseService.call(jt_move))
                            {
                                for(int j = 0; j < wam_desired_jts.size(); j++)
                                    wam_desired_jts[j] = jt_move.request.joints[j];
                                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                                {
                                    cout << "Moved : " << i << ": [";
                                    for(int j=0;j<7;j++)
                                        cout << wam_current_jts[j] << " ";
                                    cout << "]\n";

                                    cout << "service done: arm reached desired jt angles" << endl;
                                    success = true;
                                }
                                else
                                {
                                    cout << "Failed to reach desired wam position" << endl;
                                    success = false;
                                }
                            }
                            else
                            {
                                cout << "service not done: arm not reached desired jt angles" << endl;
                                success = false;

                            }
                        }
                    }



                    // Turn on the vaccum cleaner
#if(VACCUM_SUCTION)
                    if(success)
                    {
                        ros::Rate loop_rate(10);
                        std_msgs::Int16 vaccumData;
                        vaccumData.data = 1;
                        for(int i=0;i<10;i++)
                        {
                            vaccumCleanerPub.publish(vaccumData);
                            loop_rate.sleep();
                        }
                        success = true;
                        cout << "Turned on vaccum cleaner" << endl;
                    }
#endif
                    // check whether bellow cup is stuck at the metal base of bin
                    metal_data_available = false;
                    while(!metal_data_available)
                        ros::spinOnce();

#if(VACCUM_SUCTION)
                    if(metal_val > 500)// if metal detector value is greater than 1000 then turn off the vaccum cleaner
                    {
                        ros::Rate loop_rate(10);
                        std_msgs::Int16 vaccumData;
                        vaccumData.data = 2;
                        for(int i=0;i<10;i++)
                        {
                            vaccumCleanerPub.publish(vaccumData);
                            loop_rate.sleep();
                        }
                        //                    sleep(1);
                        success = true;
                        cout << "Turned off vaccum cleaner" << endl;
                        cout << "detected that bellow is stuck at the metal base" << endl;
                        metal_val = 0;
                    }

#endif


                    if(success)
                    {
                        ROS_INFO("\n Coming out of bin");
                        for(int i=0; i<traj_come_out.size(); i++)
                        {
                            if(success)
                            {
                                cout << "Coming out: " << i+1 << endl;
                                jt_move.request.joints.clear();
                                for(int j=0; j<n_jts; j++)
                                    jt_move.request.joints.push_back(traj_come_out[i][j]);

                                if(trajRpt.response.idx_bend.data.size()>1)
                                {
                                    if(i==trajRpt.response.idx_bend.data[1])
                                    {
                                        // call service to bend the pipe at eef
                                        if(1)
                                        {
                                            cout << "service done: Tool eef bent" << endl;
                                            success = true;
                                        }
                                        else
                                        {
                                            cout << "service not done: Tool eef not bent" << endl;
                                            success = false;
                                        }
                                    }
                                }

#if(ROBOT_ANGLE_SIMULATION)
                                vector<double> binPose(7);
                                for(int j=0;j<7;j++)
                                    binPose[j] = jt_move.request.joints[j];
                                publishJointStateInSimulation(binPose,simJointPublisher);
#endif
                                cout << "Coming out to: " << i << ": [";
                                for(int j=0;j<7;j++)
                                    cout << jt_move.request.joints[j] << " ";
                                cout << "]\n";

                                // keep checking if bellow is stuck at the base of the bin
                                metal_data_available = false;
                                while(!metal_data_available)
                                    ros::spinOnce();
#if(VACCUM_SUCTION)
                                if(metal_val > 1000)// if metal detector value is greater than 1000 then turn off the vaccum cleaner
                                {
                                    ros::Rate loop_rate(10);
                                    std_msgs::Int16 vaccumData;
                                    vaccumData.data = 2;
                                    for(int i=0;i<10;i++)
                                    {
                                        vaccumCleanerPub.publish(vaccumData);
                                        loop_rate.sleep();
                                    }
                                    //                                sleep(1);
                                    success = true;
                                    cout << "Turned off vaccum cleaner" << endl;
                                    cout << "detected that bellow is stuck at the metal base" << endl;
                                    metal_val = 0;
                                }
#endif
                                if(poseService.call(jt_move))
                                {
                                    for(int j = 0; j < wam_desired_jts.size(); j++)
                                        wam_desired_jts[j] = jt_move.request.joints[j];
                                    if(checkJointsReached(wam_desired_jts,wam_current_jts))
                                    {
                                        cout << "Moved to: " << i << ": [";
                                        for(int j=0;j<7;j++)
                                            cout << wam_current_jts[j] << " ";
                                        cout << "]\n";
                                        cout << "service done: arm reached desired jt angles" << endl;
                                        success = true;
                                    }
                                    else
                                    {
                                        cout << "Failed to reach desired wam position" << endl;
                                        success = false;
                                    }
                                }
                                else
                                {
                                    cout << "service not done: arm not reached desired jt angles" << endl;
                                    success = false;
                                    break;
                                }
                            }
                        }
                    }
                }

                // check whether object is still there using ir sensor data
                while(!ir_data_available)
                    ros::spinOnce();
                ir_data_available = false;


                // for objects which are in ir checking exception list consider object is there
                if(ir_exception_list[obj_id_rpt-1] == 1)
                    ir_val = 1023;

                // Go to tote drop position
                if(success && ir_val > 600) // if ir value is greater that 600 then object is still there, go to drop position
                {
#if(ROBOT)
                    apc_controller::JointMove joint_move;
                    vector<double> tote_pose(7,0.0);

                    // create trajectory to reach tote
                    vector<vector<double> > tote_traj;
                    vector<double> jt_psn(7, 0.0);
                    // copy trajectory intermediate positions
                    for(int i=0; i<1; i++)// for dropping with bin in front
                    {
                        for(int j=0; j<7; j++)
                        {
                            jt_psn[j] = drop_itd[i][j];
                        }
                        tote_traj.push_back(jt_psn);
                    }
                    if(bin_idx>=0 && bin_idx <= 5)
                    {
                        int idx = bin_idx% 3;
                        for(int j=0; j<7; j++)
                        {
                            jt_psn[j] = drop_front_r12[idx][j];
                        }
                        tote_traj.push_back(jt_psn);
                    }
                    else if(bin_idx>=6 && bin_idx<=8)
                    {
                        int idx = bin_idx% 3;
                        for(int j=0; j<7; j++)
                        {
                            jt_psn[j] = drop_front_r3[idx][j];
                        }
                        tote_traj.push_back(jt_psn);
                    }
                    else if(bin_idx>=9 && bin_idx<=11)
                    {
                        int idx = bin_idx% 3;
                        for(int j=0; j<7; j++)
                        {
                            jt_psn[j] = drop_front_r4[idx][j];
                        }
                        tote_traj.push_back(jt_psn);
                    }

                    //                // copy jt position based on bin index
                    //                if(bin_idx>=0 && bin_idx <= 5)
                    //                {
                    //                    int idx = bin_idx% 3;
                    //                    for(int j=0; j<7; j++)
                    //                    {
                    //                        jt_psn[j] = drop_l1[idx][j];
                    //                    }
                    //                    tote_traj.push_back(jt_psn);
                    //                }
                    //                else if(bin_idx>=6 && bin_idx<=8)
                    //                {
                    //                    int idx = bin_idx% 3;
                    //                    for(int j=0; j<7; j++)
                    //                    {
                    //                        jt_psn[j] = drop_l3[idx][j];
                    //                    }
                    //                    tote_traj.push_back(jt_psn);
                    //                }
                    //                else if(bin_idx>=9 && bin_idx<=11)
                    //                {
                    //                    int idx = bin_idx% 3;
                    //                    for(int j=0; j<7; j++)
                    //                    {
                    //                        jt_psn[j] = drop_l4[idx][j];
                    //                    }
                    //                    tote_traj.push_back(jt_psn);
                    //                }

                    // Give command to robot to reach the tote
                    for(int k=0; k<tote_traj.size(); k++)
                    {
                        joint_move.request.joints.clear();
                        //                    cout << "Moving to: [";
                        for(int j=0; j<7; j++)
                        {
                            joint_move.request.joints.push_back(tote_traj[k][j]);
                            tote_pose[j] = tote_traj[k][j];
                            //                        cout << tote_traj[k][j] << " ";
                        }
                        //                    cout << "]" << endl;

#if(ROBOT_ANGLE_SIMULATION)
                        publishJointStateInSimulation(tote_pose,simJointPublisher);
#endif
                        success = false;

                        if(poseService.call(joint_move))
                        {
                            for(int i = 0; i < wam_desired_jts.size(); i++)
                                wam_desired_jts[i] = joint_move.request.joints[i];
                            if(checkJointsReached(wam_desired_jts,wam_current_jts))
                            {
                                cout << "tote position reached " << k+1 << endl;
                            }
                            else
                                cout << "Failed to reach desired wam position" << endl;

                            success = true;
                        }
                        else
                        {
                            ROS_ERROR("Failed to call service");
                            success = false;
                        }
                    }

                    if(success)
                    {
                        // Turn off the vaccum cleaner
#if(VACCUM_SUCTION)
                        ros::Rate loop_rate(10);
                        std_msgs::Int16 vaccumData;
                        vaccumData.data = 2;// off
                        for(int i=0;i<10;i++)
                        {
                            vaccumCleanerPub.publish(vaccumData);
                            loop_rate.sleep();
                        }
                        success = true;
                        service_status_flag = true;
                        cout << "Turned off vaccum cleaner" << endl;
                        sleep(2);




                        // Writeing successful status of object picking in json file
                        json_maker::write_pick_status obj_write_status;
                        obj_write_status.request.pick_status.data = true;
//                        obj_write_status.request.obj_id.data = obj_id_rpt;
//                        In write_pick_status service call obj_id refers to the bin number, WE HAVE RETAINED obj_id TO REPRESENT BIN NUMBER TO AVOID MAKING CHANGES TO SRV FILE
                        obj_write_status.request.obj_id.data = bin_idx;

                        if(pick_status_write_service.call(obj_write_status))
                        {

                            cout << "Successfully written pick object status" << endl;
                        }
                        else
                        {
                            cout << "Unable to call pick status write service " << endl;
                        }
#endif
                    }
                    // send command trajectory jt angles to go back in front of rack after dropping
                    //                for(int k=tote_traj.size()-3; k>=0; k--)
                    for(int k=tote_traj.size()-2; k>=0; k--)
                    {
                        joint_move.request.joints.clear();
                        //                    cout << "Moving to: [";
                        for(int j=0; j<7; j++)
                        {
                            joint_move.request.joints.push_back(tote_traj[k][j]);
                            tote_pose[j] = tote_traj[k][j];
                            //                        cout << tote_traj[k][j] << " ";
                        }
                        //                    cout << "]" << endl;
#if(ROBOT_ANGLE_SIMULATION)
                        publishJointStateInSimulation(tote_pose,simJointPublisher);
#endif
                        success = false;

                        if(poseService.call(joint_move))
                        {
                            for(int i = 0; i < wam_desired_jts.size(); i++)
                                wam_desired_jts[i] = joint_move.request.joints[i];
                            if(checkJointsReached(wam_desired_jts,wam_current_jts))
                            {
                                cout << "tote position reached " << k+1 << endl;
                            }
                            else
                                cout << "Failed to reach desired wam position" << endl;

                            success = true;
                        }
                        else
                        {
                            ROS_ERROR("Failed to call service");
                            success = false;
                        }
#endif
                    }
                    // object is placed in tote
//                    break; // break out of the for loop



                }
                else
                {
                    // In case object was not picked or dropped while coming out detect it through ir sensor, turn off vaccum cleaner and send that object was not picked
#if(VACCUM_SUCTION)
                    ros::Rate loop_rate(10);
                    std_msgs::Int16 vaccumData;
                    vaccumData.data = 2;// off
                    for(int i=0;i<10;i++)
                    {
                        vaccumCleanerPub.publish(vaccumData);
                        loop_rate.sleep();
                    }
                    success = true;
                    service_status_flag = true;
                    cout << "Turned off vaccum cleaner" << endl;
#endif
//                    if(attempt == 1)// object is not picked even at second attempt
                    {
                        cout << "Object is not attached\n Go to next bin" << endl;

                        json_maker::write_pick_status obj_write_status;
                        obj_write_status.request.pick_status.data = false;
//                        obj_write_status.request.obj_id.data = obj_id_rpt;
//                        In write_pick_status service call obj_id refers to the bin number, WE HAVE RETAINED obj_id TO REPRESENT BIN NUMBER TO AVOID MAKING CHANGES TO SRV FILE
                        obj_write_status.request.obj_id.data = bin_idx;

                        if(pick_status_write_service.call(obj_write_status))
                        {

                            cout << "Successfully written pick object status" << endl;
                        }
                        else
                        {
                            cout << "Unable to call pick status write service " << endl;
                        }
                    }
                }
            }
        }



//        if(!service_status_flag)
//            break;

//        break;

        bin_iteration++;




    }


    // --------------------------- After completion of everything ... Reach to home position .................---------------------


    cout << "Home position call" << endl;


    publishJointStateInSimulation(start_int_pose,simJointPublisher);

    if(poseService.call(joint_srv_start_int))
    {
        cout << "Start intermediate position reached " << endl;
        for(int i = 0; i < wam_desired_jts.size(); i++)
            wam_desired_jts[i] = joint_srv_start_int.request.joints[i];
        if(checkJointsReached(wam_desired_jts,wam_current_jts))
        {
//            sleep(4);
            cout << "Start intermediate position reached " << endl;
        }
        else
            cout << "Failed to reach desired wam position" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }

    publishJointStateInSimulation(home_pose,simJointPublisher);
    if(poseService.call(joint_srv_home))
    {
        cout << "Home position reached " << endl;
        for(int i = 0; i < wam_desired_jts.size(); i++)
            wam_desired_jts[i] = joint_srv_home.request.joints[i];
        if(checkJointsReached(wam_desired_jts,wam_current_jts))
        {
//            sleep(4);
            cout << "Home position reached " << endl;
        }
        else
            cout << "Failed to reach desired wam position" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }



    return 0;
}
