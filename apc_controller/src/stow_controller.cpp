#include <iostream>
#include <apc_controller.h>


#define GRIPPER_STOP 0
#define GRIPPER_OPEN 1
#define GRIPPER_CLOSE 2

#define VACUUM_ON 5
#define VACUUM_OFF 6
// Call service to get bin contents


using namespace std;

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"stow_controller");

//    ros::NodeHandle nh;

//    ros::ServiceClient linesRackDetectService = nh.serviceClient<apc_controller::DetectShelf>("/lines_rack_detect");

//    ros::ServiceClient continuousTrajService = nh.serviceClient<apc_controller::TrajStow>("/trajectory/continuous");

////    ros::ServiceServer shelf_dataService = nh.advertiseService("/shelf_data", shelfDataCallback);

//    ros::Publisher simJointPublisher = nh.advertise<sensor_msgs::JointState>("/apc_controller/joint_states",1);

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
//    bool get_cloud = true;
//    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
//                                                                       boost::bind(kinectPCCallback, _1, boost::ref(cloud), boost::ref(get_cloud)));

//    vector<double> wam_current_jts(7);
//    vector<double> wam_desired_jts(7);

//    ros::Subscriber sub_wam_joints = nh.subscribe<sensor_msgs::JointState>
//            ("/wam/joint_states",1,boost::bind(wamJointStateCallback,_1,boost::ref(wam_current_jts)));


//    ros::ServiceClient poseService = nh.serviceClient<apc_controller::JointMove>("/wam/joint_move");
//    apc_controller::JointMove joint_srv_start,joint_srv_home, joint_srv_start_int;
//    apc_controller::JointMove joint_srv_toteC1, joint_srv_toteC2, joint_srv_toteC3;


//    ros::Publisher vaccumDataPub = nh.advertise<std_msgs::Int16>("/toggle_led",1);
//    ros::Publisher vaccumCleanerPub = nh.advertise<std_msgs::Int16>("/vacuum_topic",1);
//    int ir_val, metal_val;
//    bool ir_data_available = false, metal_data_available = false;
//    ros::Subscriber ir_sub = nh.subscribe<std_msgs::Int16>("/ir_topic", 1, boost::bind(irSensorCallback, _1, boost::ref(ir_val), boost::ref(ir_data_available)));
//    ros::Subscriber metal_sub = nh.subscribe<std_msgs::Int16>("/metal_topic", 1, boost::bind(metalSensorCallback, _1, boost::ref(metal_val), boost::ref(metal_data_available)));

//    ros::Publisher gripper_control_pub = nh.advertise<std_msgs::Int16>("/gripper_ctrl",1);

//    ros::ServiceClient stowToteContentsService = nh.serviceClient<apc_controller::stowToteContents>("/tote_contents/data");
//    ros::ServiceClient write_stow_data_service = nh.serviceClient<apc_controller::write_stow_data>("/write_stow_data_service");

//    vector<int> tote_obj(12);
//    vector<bool> tote_obj_picked(12);

//    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/bin_corner",10);


//    ifstream startPosfile;
//    std::string dataPath_start = ros::package::getPath("apc_controller").append("/data/start_pos.txt");

//    startPosfile.open(dataPath_start.c_str());
//    if(!startPosfile.is_open())
//    {
//        cerr << "Error opening file start pos" << endl;
//        exit(0);
//    }

//    ifstream startIntPosfile;
//    std::string dataPath_startInt = ros::package::getPath("apc_controller").append("/data/start_intermediate.txt");
//    startIntPosfile.open(dataPath_startInt.c_str());
//    if(!startIntPosfile.is_open())
//    {
//        cerr << "Error opening file intermediate start pos" << endl;
//        exit(0);
//    }

//    ifstream homePosFile;
//    std::string dataPath_home = ros::package::getPath("apc_controller").append("/data/home_pos.txt");

//    homePosFile.open(dataPath_home.c_str());
//    if(!homePosFile.is_open())
//    {
//        cerr << "Error opening gile" << endl;
//        exit(0);
//    }

//    //-------------Real robot movement data


//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        startPosfile >> temp;
//        joint_srv_start.request.joints.push_back(temp);
//    }

//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        startIntPosfile >> temp;
//        joint_srv_start_int.request.joints.push_back(temp);
//    }

//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        homePosFile >> temp;
//        joint_srv_home.request.joints.push_back(temp);
//    }

//    //---------------------- Simulation data
//    vector<double> home_pose;
//    vector<double> start_pose;
//    vector<double> start_int_pose;

//    startPosfile.seekg(0,std::ios_base::beg);
//    startIntPosfile.seekg(0,std::ios_base::beg);
//    homePosFile.seekg(0,std::ios_base::beg);

//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        startPosfile >> temp;
//        start_pose.push_back(temp);
//    }

//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        startIntPosfile >> temp;
//        start_int_pose.push_back(temp);
//    }

//    for(int i=0;i<7;i++)
//    {
//        double temp = 0.0;
//        homePosFile >> temp;
//        home_pose.push_back(temp);
//    }

//    // Shelf detection variables
//    apc_controller::DetectShelf dhselfCentroid;
//    vector<double> centroidArray;
//    int shelfBinIndex = 0;

//#if(XWAM_MOTION)
//    ros::ServiceClient motionFwdBckService = nh.serviceClient<apc_controller::FwdBck>("/motion/fwdbck");
//    ros::ServiceClient motionUpDownService = nh.serviceClient<apc_controller::UpDown>("/motion/updown");

//    apc_controller::FwdBck xwam_fwdback;
//    apc_controller::UpDown xwam_updown;
//    double xwam_dist_err, xwam_height_err;
//#endif

//#if(TRAJECTORY_RPT)
////    ros::ServiceClient trajRptService = nh.serviceClient<apc_controller::TrajRpt>("/trajectory/rpt");
//    ros::ServiceClient trajStowService = nh.serviceClient<apc_controller::TrajStow>("/trajectory/stow");
//#endif

//#if(RCNN)
//    ros::ServiceClient objDetectRCNNService = nh.serviceClient<apc_controller::objectDetect>("/detect_object");
//    ros::ServiceClient stowObjDetectRCNNService = nh.serviceClient<apc_controller::stowObjDetect>("/stow/detect_object");
//#endif

//    ros::ServiceClient binCropService = nh.serviceClient<apc_controller::CropBin>("/crop/bin");

//    double drop_front[][7] = {{1.524, -0.181, -0.133, 2.662, -0.003, -0.139, -0.241},
//                              {1.568, -0.155, 0.114, 2.709, 0.042, -0.208, -0.182},
//                              {1.594, -0.158, 0.459, 2.639, 0.059, -0.201, -0.173}};

////    with tote infront of robot
////    double tote_view_psn[][7] = {{0.036,  0.341, -0.057,  2.673, -0.124,  0.407,  0.011},
////                                 {0.040,  0.335, -0.006,  2.643, -0.123,  0.404,  0.011}};
////    with tote place to right of robot
//    double tote_view_psn[][7] = {{-1.62576, 0.331666, 0.0578362, 2.56212, -1.39047, 1.46796, -0.0865112}};
//    double ext_camera_psn[][7] = {{}};
//    double front_camera_show_psn[][7] = {{}};
//    double tote_to_rack[][7] = {{-0.0487587, 0.140882, 0.0296935, 1.98676, 0.253107, 0.295015, 0.0182886},
//                                {1.45809, -0.407469, 0.184729, 2.44295, 0.33708, 0.107616, 0.168502}};

//    double bin_view[][7] = {{1.523,  0.053,  0.108,  1.757, -0.542,  1.379,  0.264},
//                          {1.5015, 0.0070, 0.1092, 2.0130, -0.7270, 0.9596, 1.0590}};
//    int num_itd = 2;

//    std::string model_names[] = {

//        "dove_beauty_bar",          "rawlings_baseball",                    "clorox_utility_brush",         "dr_browns_bottle_brush",   "dasani_water_bottle",
//        "easter_turtle_sippy_cup",  "cherokee_easy_tee_shirt",              "folgers_classic_roast_coffee", "crayola_24_ct",            "peva_shower_curtain_liner",
//        "barkely_hide_bones",       "kyjen_squeakin_eggs_plush_puppies",    "expo_dry_erase_board_eraser",  "scotch_duct_tape",         "jane_eyre_dvd",
//        "scotch_bubble_mailer",     "woods_extension_cord",                 "womens_knit_gloves",           "cool_shot_glue_sticks",    "elmers_washable_no_run_school_glue",
//        "staples_index_cards",      "laugh_out_loud_joke_book",             "i_am_a_bunny_book",            "kleenex_tissue_box",       "soft_white_lightbulb",
//        "kleenex_paper_towels",     "rolodex_jumbo_pencil_cup",             "ticonderoga_12_pencils",       "platinum_pets_dog_bowl",   "hanes_tube_socks",
//        "creativity_chenille_stems","fiskars_scissors_red",                 "cloud_b_plush_bear",           "safety_first_outlet_plugs","fitness_gear_3lb_dumbbell",
//        "oral_b_toothbrush_green",  "up_glucose_bottle",                    "command_hooks",                "oral_b_toothbrush_red", "none"
//    };

//    std::string bin_names[] = {"bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"};

//    int ir_exception_list[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                               0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
//                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
//    bool service_status_flag = false;

//    double error_height = 0.0;
//    double error_motion = 0.0;

//    int bin_iteration = 0;

//    while(ros::ok())
//    {
//        signal(SIGINT, signal_callback_handler);
//        int input;
//        cout << "\n Press 0 for Start Position "
//             << "\n Press 1 for Home Position "
//             << "\n Press 2 to read the tote contents"
//             << "\n Press 3 to reach bin with tip "
//             << "\n Press 4 to reach intermediate position "
//             << "\n Press 5 go up position "
//             << "\n Press 6 to test writing of stow task json file"
//             << "\n Press 7 to test samrat's code for trajectory motion"
//             << "\n Press 9 For Stow object detection and planning "
//             << "\n Press 10 For Stow object detection test"
//             << "\n Press 11 for lines based rack corner detection"
//             << "\n Press 12 for moving robot fwd or bwd in multiples of +-0.245"
//             << "\n Press 16 to go to start position"
//             << "\n Press 17 to go in front of bin4 to check calibration"
//             << "\n Press 18 to call bin crop service"
//             << "\n Press 19 to call rack registration service"
//             << endl;
//        cin >> input;
//        getchar();

//        switch(input)
//        {
//        case 0:
//        {
//            cout << "Start position call" << endl;

//            jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");

//            jtMoveServiceCall(start_pose, simJointPublisher, poseService, wam_current_jts, "Start position reached ");
//        }
//            break;

//        case 1:
//        {
//            cout << "Home position call" << endl;

//            jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");

//            jtMoveServiceCall(home_pose, simJointPublisher, poseService, wam_current_jts, "Home position reached ");
//        }
//            break;

//        case 2:
//        {
//            // call service to get the tote contents
//            apc_controller::stowToteContents tote_contents;
//            if(stowToteContentsService.call(tote_contents))
//            {
//                cout << "Tote contents:\n";
//                for(int i=0; i<tote_contents.response.tote_contents.data.size(); i++)
//                {
//                    tote_obj[i] = tote_contents.response.tote_contents.data[i];
//                    tote_obj_picked[i] = false;

//                    cout << tote_obj[i] << " " << model_names[tote_obj[i]-1] << endl;
//                }
//                cout << "Got the tote contents" << endl;
//            }
//            else
//                cout << "Failed to call tote object content service " << endl;
//        }
//            break;
//        case 4:
//        {
//            cout << "Home position call" << endl;

//            jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");

//        }
//            break;

//        case 5:
//        {
//            // copy the jt angles for tote center view position
//            vector<double> jts(7,0.0);
//            int b;
//            cout << "Enter 0 or 1" << endl;
//            cin >> b;
//            for(int i=0; i<jts.size(); i++)
//                jts[i] = tote_view_psn[0][i];
////                jts[i] = bin_view[b][i];

//            // publish command to reach tote center view position
//            jtMoveServiceCall(jts, simJointPublisher, poseService, wam_current_jts, "tote center view position reached ");
//        }
//            break;

//        case 6:
//        {
//            for(int i=0; i<tote_obj.size(); i++)
//            {
//                if(!tote_obj_picked[i])
//                    cout << tote_obj[i] << " " << model_names[tote_obj[i]-1].c_str() << endl;
//            }
//            cout << "Enter the index of obj: " << endl;
//            int idx;
//            cin >> idx;

//            for(int i=0; i<tote_obj.size(); i++)
//            {
//                if(tote_obj[i] == idx)
//                {
//                    tote_obj_picked[i] = true;
//                    cout << model_names[tote_obj[i]-1] << " is picked" << endl;
//                    apc_controller::write_stow_data write_stow_data;
//                    write_stow_data.request.bin_id.data = i;//bin_idx;
//                    write_stow_data.request.obj_id.data = tote_obj[i];


//                    if(write_stow_data_service.call(write_stow_data))
//                    {
//                        cout << "Wrote into file " << tote_obj[i] << " is put into " << i << " bin" << endl;
//                    }
//                    else
//                        cout << "Failed to call write tote object data service" << endl;

//                    break;
//                }
//            }
//        }
//            break;

//        case 7:
//        {
//            // Make sure to execute case 11 before doing this so as to generate centroids of rack
//            apc_controller::TrajStow trajStow;
//            cout << "Enter bin num 4 or 7" << endl;
//            cin >> trajStow.request.bin_num.data;
//            getchar();
//            for(int i=0; i<3; i++)
//                trajStow.request.bin_centroid.data.push_back(centroidVector[trajStow.request.bin_num.data][i]);
//            vector<double> traj_jts;
//            int n_jts = 7;
//            cout << "Continuous trajectory service called" << endl;
//            if(continuousTrajService.call(trajStow))
//            {
//                traj_jts.resize(trajStow.response.jts_reach_rack.data.size());
//                for(int i=0; i<trajStow.response.jts_reach_rack.data.size(); i++)
//                {
//                    traj_jts[i] = trajStow.response.jts_reach_rack.data[i];
//                }
//            }
//            else
//                cout << "Failed to call contiuous trajectory service" << endl;

//            // traj_jts gives you the joint angles for all the trajectory points. Take joint angles in steps of 7 joint angles to get a traj pt
//            for(int i=0; i< traj_jts.size()/7; i++)
//            {
//                cout << "Traj pt: " << i+1 << endl;
//                for(int j=0; j<7; j++)
//                {
//                    cout << traj_jts[i*7+j] << " ";
//                }
//                cout << endl;
//            }

//        }
//            break;

//        case 10:
//        {
//            cout << "Tote object detection test" << endl;
//            bool success = true;

//            apc_controller::stowObjDetect stow_obj_detect;
//            // tote object detection service call
//            if(success)
//            {
//                for(int i=0; i<tote_obj.size(); i++)
//                {
//                    if(!tote_obj_picked[i])
//                        stow_obj_detect.request.tote_object_list.data.push_back(tote_obj[i]);
//                }

//                // copy the jt angles for tote center view position
//                vector<double> jts(7,0.0);
//                for(int i=0; i<jts.size(); i++)
//                    jts[i] = tote_view_psn[0][i];

//                // publish command to reach tote center view position
//                jtMoveServiceCall(jts, simJointPublisher, poseService, wam_current_jts, "tote center view position reached ");
//            }
//            sleep(3);// wait for robot its position before saving a cloud
//            if(success)
//            {
//                // get cloud with kinect at center position
//                get_cloud = true;
//                while(get_cloud && ros::ok())
//                {
//                    get_cloud = true;
//                    ros::spinOnce();
//                }
//                cout << "Cloud found" << endl;
//                pcl::toROSMsg(*cloud,stow_obj_detect.request.center_rgb_cloud);
//            }

//            // call service to take tote left and right view position
////            {

////            }

//            geometry_msgs::Point obj_axis, obj_normal, obj_centroid, gripping_axis, gripping_normal, gripping_centroid;
//            int bin_idx;// bin index to put the object picked from tote
//            // call service for tote object detection
//            if(success)
//            {
//                if(stowObjDetectRCNNService.call(stow_obj_detect))
//                {
//                    cout << "Found the object" << endl;

////                    cout << "Gripping axis: " << stow_obj_detect.response.gripping_axis.x << " "
////                            << stow_obj_detect.response.gripping_axis.y << " " << stow_obj_detect.response.gripping_axis.z << endl;
////                    cout << "Gripping normal: " << stow_obj_detect.response.gripping_normal.x << " "
////                            << stow_obj_detect.response.gripping_normal.y << " " << stow_obj_detect.response.gripping_normal.z << endl;
////                    cout << "Gripping centroid: " << stow_obj_detect.response.gripping_centroid.x << " "
////                            << stow_obj_detect.response.gripping_centroid.y << " " << stow_obj_detect.response.gripping_centroid.z << endl;

////                    if(stow_obj_detect.response.flag_grip.data)// if to use gripper
////                    {
////                        kinectToWAMVector(stow_obj_detect.response.gripping_axis, gripping_axis);
////                        kinectToWAMVector(stow_obj_detect.response.gripping_normal, gripping_normal);
////                        kinectToWAMBase(stow_obj_detect.response.gripping_centroid, gripping_centroid);
////                    }
////                    else//
//                    {
////                        kinectToWAMVector(stow_obj_detect.response.obj_axis, obj_axis);
//                        kinectToWAMVector(stow_obj_detect.response.obj_normal, obj_normal);
//                        kinectToWAMBase(stow_obj_detect.response.obj_centroid, obj_centroid);
//                    }
//                    success = true;
//                }
//                else
//                {
//                    ROS_ERROR("Failed to call tote object detection service");
//                    success = false;
//                }
//            }
//            // trajectory planning service call
//            apc_controller::TrajStow traj_stow;
//            if(success)
//            {
////                traj_stow.request.flag_grip.data = false;//stow_obj_detect.response.flag_grip.data;
////                traj_stow.request.gripping_axis = gripping_axis;
////                traj_stow.request.gripping_normal = gripping_normal;
////                traj_stow.request.gripping_centroid = gripping_centroid;
////                traj_stow.request.obj_axis = obj_axis;
//                traj_stow.request.obj_normal = obj_normal;
//                traj_stow.request.obj_centroid = obj_centroid;
//                traj_stow.request.obj_id.data = stow_obj_detect.response.recognized_obj_id.data;
//                traj_stow.request.obj_psn_suction.data = stow_obj_detect.response.obj_psn_suction.data;
////                traj_stow.request.flag_grip.data = stow_obj_detect.response.flag_grip.data;
//                bin_idx = 7;// specify the bin idx where the object has to be stowed
//                for(int j=0; j<3; j++)
//                    traj_stow.request.bin_centroid.data.push_back(centroidVector[bin_idx][j]);
//                traj_stow.request.bin_num.data = bin_idx;


//                if(trajStowService.call(traj_stow))
//                {
//                    cout << "Obtained trajectory planning" << endl;
////                    cout << "Centroid: \n";
////                    if(stow_obj_detect.response.flag_grip.data)
////                    {
////                        cout << stow_obj_detect.response.gripping_centroid.x << " " << stow_obj_detect.response.gripping_centroid.y << " "
////                             << stow_obj_detect.response.gripping_centroid.z << endl;
////                    }
////                    else
////                    {
////                        cout << stow_obj_detect.response.obj_centroid.x << " " << stow_obj_detect.response.obj_centroid.y << " "
////                             << stow_obj_detect.response.obj_centroid.z << endl;
////                    }
//                    success = true;
//                }
//                else
//                {
//                    ROS_ERROR("Failed to call trajectory planning service");
//                    success = false;
//                }
//            }

//            // execute the planned trajectory to reach and pick the object
//            if(success)
//            {
//                // copy the trajectory joint angle values into the local variables
//                int n_jts = 7;// Number of joints on arm
//                int n_trj = traj_stow.response.jts_reach_tote.data.size()/n_jts;
//                vector< vector<double> > traj_reach_obj = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

//                n_trj = traj_stow.response.jts_back_tote.data.size()/n_jts;
//                vector< vector<double> > traj_come_out = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

//                n_trj = traj_stow.response.jts_reach_rack.data.size()/n_jts;
//                vector< vector<double> > traj_rack = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

//                vector< vector<double> > traj_itd = vector < vector<double> >(num_itd, vector<double>(n_jts,0));

//                cout << "Copy Trajectory angles to object in tote: " << traj_reach_obj.size() << "\n[";
//                int r = 0,s=0;
//                for(vector<double>::const_iterator it = traj_stow.response.jts_reach_tote.data.begin();
//                    it!=traj_stow.response.jts_reach_tote.data.end(); ++it)
//                {
//                    traj_reach_obj[r][s] = *it;
//                    s++;
//                    if(s==n_jts)
//                    {
//                        r++;
//                        s=0;
//                    }
//                }
//                cout << "Copy Trajectory angles to come out of tote: " << traj_come_out.size() << "\n ";
//                r = 0,s=0;
//                for(vector<double>::const_iterator it = traj_stow.response.jts_back_tote.data.begin();
//                    it!=traj_stow.response.jts_back_tote.data.end(); ++it)
//                {
//                    traj_come_out[r][s] = *it;
//                    s++;
//                    if(s==n_jts)
//                    {
//                        r++;
//                        s=0;
//                    }
//                }
//                cout << "Copy Trajectory to reach rack angles: " << traj_rack.size() << "\n ";
//                r = 0,s=0;
//                for(vector<double>::const_iterator it = traj_stow.response.jts_reach_rack.data.begin();
//                    it!=traj_stow.response.jts_reach_rack.data.end(); ++it)
//                {
//                    traj_rack[r][s] = *it;
//                    s++;
//                    if(s==n_jts)
//                    {
//                        r++;
//                        s=0;
//                    }
//                }

//                // copy joint angles to go from tote to in front of rack
//                for(int i=0; i<num_itd; i++)
//                {
//                    for(int j=0; j<n_jts; j++)
//                        traj_itd[i][j] = tote_to_rack[i][j];
//                }

////                if(!traj_stow.response.flag_grip.data)// if to use suction then close the gripper teeths
////                {
////                    // to turn on suction close the gripper teeths
////                    ros::Rate loop_rate(10);
////                    std_msgs::Int16 gripperData;
////                    gripperData.data = GRIPPER_CLOSE;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                    sleep(8);// wait time before gripper motion completes
////                    gripperData.data = GRIPPER_STOP;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                }

////                cout << "Make sure gripper is open" << endl;
//                char cc;
////                cin >> cc;
////                getchar();

//                for(int i=0; i<traj_reach_obj.size(); i++)
//                {
////                    reach the object and turn on the suction or close the gripper
//                    if(success)
//                    {
//                        char s[10];
//                        sprintf(s," %d", i);
//                        string str = "Reached traj position step:";
//                        str.append(s);
//                        cout << "Moving to: " << i << ": [";
//                        for(int j=0;j<7;j++)
//                            cout << traj_reach_obj[i][j] << " ";
//                        cout << "]\n";

//                        char c;
//                        if(i==0 || i==1)
//                        {
//                            cout << "Press 'c': " ;
//                            cin >> c;
//                            getchar();
//                        }
//                        jtMoveServiceCall(traj_reach_obj[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
//                    }
//                }

////                if(traj_stow.response.flag_grip.data)// if to use gripper
////                {
////                    // close the gripper
////                    ros::Rate loop_rate(10);
////                    std_msgs::Int16 gripperData;
////                    gripperData.data = GRIPPER_CLOSE;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                    sleep(10);// wait time before gripper motion completes
////                    gripperData.data = GRIPPER_STOP;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                }
////                else
////                {
////                    // turn on suction
////                    ros::Rate loop_rate(10);
////                    std_msgs::Int16 gripperData;
////                    gripperData.data = VACUUM_ON;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                    sleep(2);// wait time before gripper motion completes
////                }

////                cout << "Turn on vacuum cleaner" << endl;
//                // Turn on the vaccum cleaner
//#if(VACCUM_SUCTION)
//                if(success)
//                {
//                    ros::Rate loop_rate(10);
//                    std_msgs::Int16 vaccumData;
//                    vaccumData.data = 1;
//                    for(int i=0;i<10;i++)
//                    {
//                        vaccumCleanerPub.publish(vaccumData);
//                        loop_rate.sleep();
//                    }
//                    success = true;
//                    cout << "Turned on vaccum cleaner" << endl;
//                }
//#endif
//                cin >> cc;
//                getchar();

////                     pick out the object from tote
//                if(success)
//                {
//                    for(int i=0; i<traj_come_out.size(); i++)
//                    {
//                        char s[10];
//                        sprintf(s," %d", i);
//                        string str = "Coming out traj position step:";
//                        str.append(s);
//                        cout << "Moving to: " << i << ": [";
//                        for(int j=0;j<7;j++)
//                            cout << traj_come_out[i][j] << " ";
//                        cout << "]\n";

//                        jtMoveServiceCall(traj_come_out[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
//                    }
//                }

//                // go infront of bin
////                if(success)
////                {
////                    for(int i=0; i<traj_itd.size(); i++)
////                    {
////                        char s[10];
////                        sprintf(s," %d", i);
////                        string str = "Going in front of rack step:";
////                        str.append(s);
////                        cout << "Moving to: " << i << ": [";
////                        for(int j=0;j<7;j++)
////                            cout << traj_itd[i][j] << " ";
////                        cout << "]\n";

////                        jtMoveServiceCall(traj_itd[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
////                    }
////                    for(int i=0; i<traj_rack.size(); i++)
////                    {
////                        char s[10];
////                        sprintf(s," %d", i);
////                        string str = "Going inside rack step:";
////                        str.append(s);
////                        cout << "Moving to: " << i << ": [";
////                        for(int j=0;j<7;j++)
////                            cout << traj_rack[i][j] << " ";
////                        cout << "]\n";

////                        jtMoveServiceCall(traj_rack[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
////                    }
////                }


////                if(success)
////                {
//                // Turn on the vaccum cleaner
//#if(VACCUM_SUCTION)
//                if(success)
//                {
//                    ros::Rate loop_rate(10);
//                    std_msgs::Int16 vaccumData;
//                    vaccumData.data = 2;
//                    for(int i=0;i<10;i++)
//                    {
//                        vaccumCleanerPub.publish(vaccumData);
//                        loop_rate.sleep();
//                    }
//                    success = true;
//                    cout << "Turned off vaccum cleaner" << endl;
//                }
//#endif
////                   cout << "Turned off the vacuum cleaner" << endl;

////                   // come out of the rack
////                   for(int i=traj_rack.size()-2; i>=0; i++)
////                   {
////                       char s[10];
////                       sprintf(s," %d", i);
////                       string str = "Come out of rack step:";
////                       str.append(s);
////                       cout << "Moving to: " << i << ": [";
////                       for(int j=0;j<7;j++)
////                           cout << traj_rack[i][j] << " ";
////                       cout << "]\n";

////                       jtMoveServiceCall(traj_rack[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
////                   }

//                   for(int i=0; i<tote_obj.size(); i++)
//                   {
//                       if(tote_obj[i] == stow_obj_detect.response.recognized_obj_id.data)
//                       {
//                           tote_obj_picked[i] = true;
//                           cout << model_names[tote_obj[i]-1] << " is picked" << endl;
//                           apc_controller::write_stow_data write_stow_data;
//                           write_stow_data.request.bin_id.data = bin_idx;
//                           write_stow_data.request.obj_id.data = tote_obj[i];


//                           if(write_stow_data_service.call(write_stow_data))
//                           {
//                               cout << "Wrote into file " << tote_obj[i] << " is put into " << bin_names[bin_idx] << endl;
//                               success = true;
//                           }
//                           else
//                               cout << "Failed to call write tote object data service" << endl;

//                           break;
//                       }
//                   }

////                }




////                // go to external camera view position
////                vector<double> jts(7,0.0);
////                apc_controller::JointMove jt_move;
////                for(int i=0; i<jts.size(); i++)
////                {
////                    jts[i] = ext_camera_psn[0][i];
////                    jt_move.request.joints.push_back(jts[i]);
////                }

////                // publish command to reach tote center view position
////                jtMoveServiceCall(jts, simJointPublisher, poseService, wam_current_jts, "external camera view position reached ");

//                // call a service to detect object
////                {

////                }
////                if(traj_stow.response.flag_grip.data)// if to use gripper
////                {
////                    // open the gripper
////                    ros::Rate loop_rate(10);
////                    std_msgs::Int16 gripperData;
////                    gripperData.data = GRIPPER_OPEN;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                    sleep(10);// wait time before gripper motion completes
////                }
////                else
////                {
////                    // turn off suction
////                    ros::Rate loop_rate(10);
////                    std_msgs::Int16 gripperData;
////                    gripperData.data = VACUUM_OFF;
////                    for(int i=0;i<10;i++)
////                    {
////                        gripper_control_pub.publish(gripperData);
////                        loop_rate.sleep();
////                    }
////                    sleep(2);// wait time before gripper motion completes
////                }
////                cout << "Open the gripper" << endl;
////                cin >> cc;
////                getchar();



//            }

//            break;
//        }

//        case 11:
//        {
//            bool success = false;

//            cout << "Lines Rack detection call" << endl;
//            if(linesRackDetectService.call(dhselfCentroid))
//            {
//                success = true;
//                centroidArray.clear();
//                for(std::vector<double>::const_iterator it = dhselfCentroid.response.centroids.data.begin();
//                    it != dhselfCentroid.response.centroids.data.end();++it)
//                {
//                    centroidArray.push_back(*it);
//                }

//                int b = 0;
//                for(std::vector<double>::const_iterator it = dhselfCentroid.response.bin_corners.data.begin();
//                    it < dhselfCentroid.response.bin_corners.data.end(); it+=3)
//                {
//                    double x = *it;
//                    double y = *(it+1);
//                    double z = *(it+2);
//                    tf::Vector3 v(x,y,z);
//                    bin_corners[b] = v;
//                    b++;
//                }
//                for(int i = 0; i<bin_corners.size(); i+=4)
//                {
//                    cout << "Corners of Bin: " << i/4 << endl;
//                    for(int j = 0; j<4; j++)
//                        cout << "[" << bin_corners[i+j].getX() << "," << bin_corners[i+j].getY() << "," << bin_corners[i+j].getZ() << "]" << endl;
//                    cout << endl;
//                }
//                cout << "Copied Bin Corners" << endl;

//                int count = 0;
//                if(centroidArray.size() > 0)
//                {
//                    for(int i=0;i<int(NO_OF_BINS);i++)
//                    {
//                        centroidVector[i][0] = centroidArray[count];
//                        centroidVector[i][1] = centroidArray[count+1];
//                        centroidVector[i][2] = centroidArray[count+2];
//                        count += 3;
//                        cout << "Centroid of Bin: " << i << endl;
//                        cout << "[" << centroidVector[i][0] << " " << centroidVector[i][1] << " " << centroidVector[i][2] << "]" << endl;
//                    }
//                    cout << "Copied Bin Centroids" << endl;
//                }
//                cout << "lines rack detect Service done " << endl;

//                got_shelf_data = true;
//            }
//            else
//            {
//                success = false;
//                cout << "lines rack detect service not done" << endl;
//            }

//            break;
//        }


//        }// switch bracket close
//    }

    return 0;
}
