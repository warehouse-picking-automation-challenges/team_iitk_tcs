#include <iostream>
#include <apc_controller.h>

using namespace std;

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}
#define NO_OBJ_TOTE 12
#define IR_SENSOR_VAL 500

#define COMMENT 0

int main(int argc, char **argv)
{
    ros::init(argc,argv,"stow_controller");

    ros::NodeHandle nh;

    ros::ServiceClient linesRackDetectService = nh.serviceClient<apc_controller::DetectShelf>("/lines_rack_detect");

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

    ros::Publisher vaccumCleanerPub = nh.advertise<std_msgs::Int16>("/vacuum_topic",1);
    int ir_val;
    bool ir_data_available = false;
    ros::Subscriber ir_sub = nh.subscribe<std_msgs::Int16>("/ir_topic", 1, boost::bind(irSensorCallback, _1, boost::ref(ir_val), boost::ref(ir_data_available)));

    ros::ServiceClient stowToteContentsService = nh.serviceClient<apc_controller::stowToteContents>("/tote_contents/data");
    ros::ServiceClient write_stow_data_service = nh.serviceClient<apc_controller::write_stow_data>("/write_stow_data_service");
    ros::ServiceClient get_bin_content_Service = nh.serviceClient<apc_controller::getBinContents>("/get_bin_contents");

    vector<int> tote_obj(NO_OBJ_TOTE);
    vector<bool> tote_obj_picked(NO_OBJ_TOTE);

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

    //---------------------- Simulation data
    vector<double> home_pose;
    vector<double> start_pose;
    vector<double> start_int_pose;

    startPosfile.seekg(0,std::ios_base::beg);
    startIntPosfile.seekg(0,std::ios_base::beg);
    homePosFile.seekg(0,std::ios_base::beg);

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

    // Shelf detection variables
    apc_controller::DetectShelf dhselfCentroid;
    vector<double> centroidArray;
    int shelfBinIndex = 0;

#if(XWAM_MOTION)
    ros::ServiceClient motionFwdBckService = nh.serviceClient<apc_controller::FwdBck>("/motion/fwdbck");
    ros::ServiceClient motionUpDownService = nh.serviceClient<apc_controller::UpDown>("/motion/updown");

    apc_controller::FwdBck xwam_fwdback;
    apc_controller::UpDown xwam_updown;
    double xwam_dist_err, xwam_height_err;
#endif

#if(TRAJECTORY_RPT)
    ros::ServiceClient trajStowService = nh.serviceClient<apc_controller::TrajStow>("/trajectory/stow");
#endif

#if(RCNN)
    ros::ServiceClient objDetectRCNNService = nh.serviceClient<apc_controller::objectDetect>("/detect_object");
    ros::ServiceClient stowObjDetectRCNNService = nh.serviceClient<apc_controller::stowObjDetect>("/stow/detect_object");
#endif





    //    ros::ServiceClient binCropService = nh.serviceClient<apc_controller::CropBin>("/crop/bin");


    //    with tote infront of robot
    //    double tote_view_psn[][7] = {{0.036,  0.341, -0.057,  2.673, -0.124,  0.407,  0.011},
    //                                 {0.040,  0.335, -0.006,  2.643, -0.123,  0.404,  0.011}};
    //    with tote place to back of robot
    double tote_view_psn[][7] = {{-1.62576, 0.331666, 0.0578362, 2.56212, -1.39047, 1.46796, -0.0865112}};
    double ext_camera_psn[][7] = {{}};

    double tote_to_rack[][7] = {{-0.0487587, 0.140882, 0.0296935, 1.98676, 0.253107, 0.295015, 0.0182886},
                                {1.45809, -0.407469, 0.184729, 2.44295, 0.33708, 0.107616, 0.168502}};
    int num_itd = 2;

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

    std::string bin_names[] = {"bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"};

    int ir_exception_list[] = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};


    bool success = false;

    //    double error_height = 0.0;
    //    double error_motion = 0.0;

    // number of objects that can be put in each bin(bin A to bin L)
    int n_obj_bin[12] = {0, 0, 0, 1, 2, 1, 1, 2, 1, 1, 2, 1};
    //    // flag whether each object is tried once for recognition and pick up from tote
    //    vector<bool> tried_once(12, false);
    //    // flag whether all objects in the tote are tried once or not
    //    bool tried_all_once = false;

    // bin location that a tote object needs to be put into based on its size
    // 0: not so big can be put in side columns
    // 1: big enough to be put in middle column
    // 2: long objects which should be put side columns
    // 3: cloth or heavier object which should be put in lower middle bin so that they drop from below easily
    // 4: folgers to be put in 7 bin
    int bin_drop_suggestion[40] = {0, 0, 1, 1, 1,
                                   0, 3, 3, 0, 1,
                                   0, 0, 0, 0, 1,
                                   3, 0, 0, 0, 0,
                                   0, 1, 3, 1, 0,
                                   1, 1, 1, 0, 1,
                                   3, 0, 0, 0, 1,
                                   0, 1, 0, 0, 0
                                  };
    // exception list of the objects that has to be skipped from picking
    // 0: not skipped from picking
    // 1: skipped from picking
    int obj_exception_list[40] = {0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0,
                                  1, 1, 0, 0, 1,
                                  0, 1, 0, 0, 1,
                                  0, 0, 0, 0, 1
                                 };

    // in case a bin in particular column is filled but still needs to be put in a particular column then keep track of row number
    int middle_row = 1, left_row = 1, right_row = 1;
    int last_row_big = 9;
    bool left_scanned = false;// flag whether once objects have been at put inside left column bins


    int num_obj_tote;

    // call service to get the tote contents
    while(!success)
    {
        apc_controller::stowToteContents tote_contents;
        if(stowToteContentsService.call(tote_contents))
        {
            cout << "Tote contents:\n";
            for(int i=0; i<tote_contents.response.tote_contents.data.size(); i++)
            {
                tote_obj[i] = tote_contents.response.tote_contents.data[i];
                tote_obj_picked[i] = false;

                cout << tote_obj[i] << " " << model_names[tote_obj[i]-1] << endl;
            }
            num_obj_tote = tote_obj.size();
            cout << "Got the tote contents" << endl;
            success = true;
        }
        else
            cout << "Failed to call tote object content service " << endl;


    }


    vector<bool> bin_blacklist_status(12,false);
    vector<int> current_bin_items_number(12);
    int MAX_BIN_CONTENTS = 6;

    vector< vector<int> > bin_contents_ids;
    success = false;
    while(!success)
    {

        apc_controller::getBinContents get_bin_content_variable;

        if(get_bin_content_Service.call(get_bin_content_variable))
        {
            success = true;
            cout << "Getting bin contents for stowing " << endl;
            vector<int> ids;

            // push empty ids on first 3
            bin_contents_ids.push_back(ids);
            current_bin_items_number[0] = ids.size();
            ids.clear();

            bin_contents_ids.push_back(ids);
            current_bin_items_number[1] = ids.size();
            ids.clear();

            bin_contents_ids.push_back(ids);
            current_bin_items_number[2] = ids.size();
            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_d.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_d.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[3] = ids.size();
            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_e.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_e.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[4] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_f.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_f.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[5] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_g.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_g.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[6] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_h.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_h.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[7] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_i.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_i.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[8] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_j.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_j.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[9] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_k.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_k.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[10] = ids.size();

            ids.clear();

            for(int i=0; i<get_bin_content_variable.response.bin_content_l.data.size(); i++)
                ids.push_back(get_bin_content_variable.response.bin_content_l.data[i]);
            bin_contents_ids.push_back(ids);
            current_bin_items_number[11] = ids.size();

            ids.clear();

//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_d.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_e.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_f.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_g.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_h.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_i.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_j.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_k.data);
//            bin_contents_ids.push_back(get_bin_content_variable.response.bin_content_l.data);

            for(int i=3;i<12;i++)
            {
                cout << bin_names[i] << endl;
                for(int j=0;j<bin_contents_ids[i].size();j++)
                {
                    cout << model_names[bin_contents_ids[i][j]] << "\t";
                }
                cout << endl;
            }


        }
        else
            cout << "Can not call bin contents for stowing" << endl;


    }




    int paper_towel_bin_nu = -1;
    bool paper_towel_exist = false;
    // Check paper towel bin number
    for(int i=3;i<12;i++)
    {
        for(int j=0;j< bin_contents_ids[i].size();j++)
        {
            if(bin_contents_ids[i][j] == 25)// paper towel id is 25
            {
                paper_towel_bin_nu = i;
                paper_towel_exist = true;
                break;
            }

        }
    }

    cout << "Paper towel exist \t" << paper_towel_exist << endl;
//    exit(0);

    // Mark objects picked that are in exception list for not picking
    for(int i=0; i<tote_obj.size(); i++)
    {
        if(obj_exception_list[tote_obj[i]-1] == 1)
            tote_obj_picked[i] = true;
    }
    // flag whether first time(starting). If first time then do rack detection and save the centroids
    bool first_time = true;

    // go to start position
    cout << "Start position call" << endl;
    jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");
    jtMoveServiceCall(start_pose, simJointPublisher, poseService, wam_current_jts, "Start position reached ");

    bool retry_recog = false;

    char c;
    cout << "press c to start autonoumous operations" << endl;
    cin >> c;
    getchar();

    // flag whether operation can be stopped or not
    bool stow_over = false;
    success = false;





    while(ros::ok() && !stow_over)
    {
        if(first_time)
        {
            first_time = false;
            // go to start position
            jtMoveServiceCall(start_pose, simJointPublisher, poseService, wam_current_jts, "Start position reached ");

            // do rack detection and get the bin centroids
            success = false;

            while(!success)
            {
                cout << "Lines Rack detection call" << endl;
                if(linesRackDetectService.call(dhselfCentroid))
                {
                    success = true;
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
                    for(int i = 0; i<bin_corners.size(); i+=4)
                    {
                        cout << "Corners of Bin: " << i/4 << endl;
                        for(int j = 0; j<4; j++)
                            cout << "[" << bin_corners[i+j].getX() << "," << bin_corners[i+j].getY() << "," << bin_corners[i+j].getZ() << "]" << endl;
                        cout << endl;
                    }
                    cout << "Copied Bin Corners" << endl;

                    int count = 0;
                    if(centroidArray.size() > 0)
                    {
                        for(int i=0;i<int(NO_OF_BINS);i++)
                        {
                            centroidVector[i][0] = centroidArray[count];
                            centroidVector[i][1] = centroidArray[count+1];
                            centroidVector[i][2] = centroidArray[count+2];
                            count += 3;
                            cout << "Centroid of Bin: " << i << endl;
                            cout << "[" << centroidVector[i][0] << " " << centroidVector[i][1] << " " << centroidVector[i][2] << "]" << endl;
                        }
                        cout << "Copied Bin Centroids" << endl;
                    }
                    cout << "lines rack detect Service done " << endl;

                    got_shelf_data = true;
                }
                else
                {
                    success = false;
                    cout << "lines rack detect service not done" << endl;
                }
            }

            // go to intermediate position
            //            jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");
        }

        // go to tote object detection and planning stage
        success = true;

        apc_controller::stowObjDetect stow_obj_detect;
        // tote object detection service call
        if(success)
        {
            for(int i=0; i<tote_obj.size(); i++)
            {
                if(!tote_obj_picked[i])
                    stow_obj_detect.request.tote_object_list.data.push_back(tote_obj[i]);
            }

            vector<double> jts(7,0.0);
            // copy the jt angles to take arm front of robot before taking it to tote center view position
            if(!retry_recog)// if it is not retry of recognition then move the joint positions
            {
                for(int i=0; i<jts.size(); i++)
                    jts[i] = tote_to_rack[0][i];

                // publish command to reach tote center view position
                jtMoveServiceCall(jts, simJointPublisher, poseService, wam_current_jts, "tote itd 0 position reached ");
            }

            for(int i=0; i<jts.size(); i++)
                jts[i] = tote_view_psn[0][i];
            // publish command to reach tote center view position
            jtMoveServiceCall(jts, simJointPublisher, poseService, wam_current_jts, "tote center view position reached ");

            sleep(3);// wait for robot its position before saving a cloud
        }

        if(success)
        {
            // get cloud with kinect at center position
            get_cloud = true;
            while(get_cloud && ros::ok())
            {
                get_cloud = true;
                ros::spinOnce();
            }
            cout << "Cloud found" << endl;
            pcl::toROSMsg(*cloud,stow_obj_detect.request.center_rgb_cloud);
        }
        //        cout << "Point cloud size: " << sto
        geometry_msgs::Point obj_normal, obj_centroid;// obj_axis

        int bin_idx;// bin index to put the object picked from tote
        // call service for tote object detection
        if(success)
        {
            if(stowObjDetectRCNNService.call(stow_obj_detect))
            {
                cout << "Found the object" << endl;

                {
                    //                        kinectToWAMVector(stow_obj_detect.response.obj_axis, obj_axis);
                    kinectToWAMVector(stow_obj_detect.response.obj_normal, obj_normal);
                    kinectToWAMBase(stow_obj_detect.response.obj_centroid, obj_centroid);
                }
                success = true;
                retry_recog = false;
            }
            else
            {
                ROS_ERROR("Failed to call tote object detection service");
                success = false;
                retry_recog = true;
            }
        }
        // trajectory planning service call
        apc_controller::TrajStow traj_stow;
        if(success)
        {
            //                traj_stow.request.obj_axis = obj_axis;
            traj_stow.request.obj_normal = obj_normal;
            traj_stow.request.obj_centroid = obj_centroid;
            traj_stow.request.obj_id.data = stow_obj_detect.response.recognized_obj_id.data;
            traj_stow.request.obj_psn_suction.data = stow_obj_detect.response.obj_psn_suction.data;

            // find the bin idx where the object has to be stowed
            bool found_bin_idx = false;
            int obj_idx = stow_obj_detect.response.recognized_obj_id.data;
            // check if drop suggestion is 3 then put in bin 10
            if(bin_drop_suggestion[obj_idx-1] == 3)
            {
                if(obj_idx == 16 || obj_idx == 31)// in case of bubble mailer or chenille stems
                {
                    if(paper_towel_bin_nu == 10)
                    {
                        if(bin_contents_ids[9].size() >= bin_contents_ids[11].size())
                        {
                            if(!bin_blacklist_status[9])
                            {
                                bin_idx = 9;
                                found_bin_idx = true;
                                cout << "-------------------------------" << endl;
                                cout << "Biggest obj, bin: " << bin_names[bin_idx] << endl;
                                cout << "-------------------------------" << endl;
                            }
                        }
                        else
                        {
                            if(!bin_blacklist_status[11])
                            {
                                bin_idx = 11;
                                found_bin_idx = true;
                                cout << "-------------------------------" << endl;
                                cout << "Biggest obj, bin: " << bin_names[bin_idx] << endl;
                                cout << "-------------------------------" << endl;
                            }
                        }
                    }
                    else
                    {
                        if(obj_idx == 16)// in case of bubble mailer
                        {
                            found_bin_idx = true;
                            bin_idx = 10;// select bin 10 as the place to put object
                            cout << "-------------------------------" << endl;
                            cout << "Bubble mailer obj, bin: " << bin_names[bin_idx] << endl;
                            cout << "-------------------------------" << endl;
                        }
                        else if(bin_contents_ids[9].size() >= bin_contents_ids[11].size() &&
                                bin_contents_ids[9].size() >= bin_contents_ids[10].size())
                        {
                            if(!bin_blacklist_status[9])
                            {
                                bin_idx = 9;
                                found_bin_idx = true;
                                cout << "-------------------------------" << endl;
                                cout << "Biggest obj, bin: " << bin_names[bin_idx] << endl;
                                cout << "-------------------------------" << endl;
                            }
                        }
                        else if(bin_contents_ids[11].size() >= bin_contents_ids[9].size() &&
                                bin_contents_ids[11].size() >= bin_contents_ids[10].size())
                        {
                            if(!bin_blacklist_status[11])
                            {
                                bin_idx = 11;
                                found_bin_idx = true;
                                cout << "-------------------------------" << endl;
                                cout << "Biggest obj, bin: " << bin_names[bin_idx] << endl;
                                cout << "-------------------------------" << endl;
                            }
                        }

//                        found_bin_idx = true;
//                        bin_idx = 10;// select bin 10 as the place to put object
//                        cout << "-------------------------------" << endl;
//                        cout << " obj, bin: " << bin_names[bin_idx] << endl;
//                        cout << "-------------------------------" << endl;
                    }
                }





//                //                bin_idx = 10;// select bin 10 as the place to put object
//                //                found_bin_idx = true;
//                //                n_obj_bin[bin_idx] -= 1;

//                if(obj_idx == 16 || obj_idx == 31)// in case of bubble mailer or chenille stems
//                {
//                    bin_idx = 10;// select bin 10 as the place to put object
//                }
//                else
//                {
//                    bin_idx = last_row_big;// select bin 10 as the place to put object
//                    if(last_row_big == 9)
//                        last_row_big = 11;
//                    else
//                        last_row_big = 9;
//                    found_bin_idx = true;
//                    n_obj_bin[bin_idx] -= 1;
//                }
            }
            // check if drop suggestion is 4 then put in bin 7
            if(bin_drop_suggestion[obj_idx-1] == 4)
            {
                if(bin_contents_ids[7].size() >= bin_contents_ids[10].size() && paper_towel_bin_nu != 7)
                {
                    if(!bin_blacklist_status[7])
                    {
                        found_bin_idx = true;
                        bin_idx = 7;
                        cout << "-------------------------------" << endl;
                        cout << "Big to put in middle obj, bin: " << bin_names[bin_idx] << endl;
                        cout << "-------------------------------" << endl;
                    }
                }
                else if(bin_contents_ids[7].size() < bin_contents_ids[10].size() && paper_towel_bin_nu != 10)
                {
                    if(!bin_blacklist_status[10])
                    {
                        found_bin_idx = true;
                        bin_idx = 10;
                        cout << "-------------------------------" << endl;
                        cout << "Big to put in middle obj, bin: " << bin_names[bin_idx] << endl;
                        cout << "-------------------------------" << endl;
                    }
                }

//                bin_idx = 7;// select bin 7 as the place to put object
//                found_bin_idx = true;
//                n_obj_bin[bin_idx] -= 1;
            }
            // fill the middle column and then go for side columns
//            if(!found_bin_idx)
//            {
//                for(int i=0; i<2; i++)// scan through middle column from row 1 to row 2 to pick a bin
//                {
//                    if(n_obj_bin[3*(i+1)+1] > 0)
//                    {
//                        bin_idx = 3*(i+1)+1;
//                        n_obj_bin[bin_idx] -= 1;
//                        found_bin_idx = true;
//                        break;
//                    }
//                }
//            }
            if(!found_bin_idx)
            {
                // check if object is big or cloth which has to be put in middle bin
                if(bin_drop_suggestion[obj_idx-1] == 1)
                {
                    if(bin_contents_ids[4].size() >= bin_contents_ids[7].size() && paper_towel_bin_nu != 4)
                    {
                        if(!bin_blacklist_status[4])
                        {
                            found_bin_idx = true;
                            bin_idx = 4;
                            cout << "-------------------------------" << endl;
                            cout << "Selection bet 4& 7 to put in middle obj, bin: " << bin_names[bin_idx] << endl;
                            cout << "-------------------------------" << endl;
                        }
                    }
                    else if(bin_contents_ids[4].size() < bin_contents_ids[7].size() && paper_towel_bin_nu != 7)
                    {
                        if(!bin_blacklist_status[7])
                        {
                            found_bin_idx = true;
                            bin_idx = 7;
                            cout << "-------------------------------" << endl;
                            cout << "Selection bet 4& 7 to put in middle obj, bin: " << bin_names[bin_idx] << endl;
                            cout << "-------------------------------" << endl;
                        }
                    }
                }
                else if(bin_drop_suggestion[obj_idx-1] == 0)
                {
                    int select_bin=3, max_obj=bin_contents_ids[3].size();
                    int second_max=bin_contents_ids[3].size(), second_max_bin=3;
                    if(bin_blacklist_status[3])
                    {
                       select_bin=4;
                       max_obj=bin_contents_ids[4].size();
                       second_max=bin_contents_ids[4].size();
                       second_max_bin=4;
                    }

                    for(int i=4; i<12; i++)
                    {
                        if(bin_contents_ids[i].size() > max_obj && !bin_blacklist_status[i])
                        {
                            // update second max select
                            second_max_bin = select_bin;
                            second_max = bin_contents_ids[second_max_bin].size();

                            // update with most highest obj numbers bin
                            max_obj = bin_contents_ids[i].size();
                            select_bin = i;
                        }
                    }
                    if(paper_towel_bin_nu != select_bin)
                    {
                        bin_idx = select_bin;
                        found_bin_idx = true;
                    }
                    else
                    {
                        bin_idx = second_max_bin;
                        found_bin_idx = true;
                    }

                    cout << "-------------------------------" << endl;
                    cout << "small obj, bin: " << bin_names[bin_idx] << endl;
                    cout << "-------------------------------" << endl;
                }

                if(!found_bin_idx)
                {
                    if(paper_towel_bin_nu != 7)
                    {
                        bin_idx = 7;
                        found_bin_idx = true;
                        cout << "-------------------------------" << endl;
                        cout << "default bin: " << bin_names[bin_idx] << endl;
                        cout << "-------------------------------" << endl;
                    }
                    else
                    {
                        bin_idx = 10;
                        found_bin_idx = true;
                        cout << "-------------------------------" << endl;
                        cout << "default bin: " << bin_names[bin_idx] << endl;
                        cout << "-------------------------------" << endl;
                    }
                }

                cout << "-------------------------------" << endl;
                cout << "Selected bin index is: " << bin_names[bin_idx] << endl;
                cout << "-------------------------------" << endl;

//                // check if object is big or cloth which has to be put in middle bin
//                if(bin_drop_suggestion[obj_idx-1] == 1)// in case big object that needs to be put into middle column
//                {
//                    bin_idx = 3*middle_row+1;
//                    middle_row++;// update to next row in case to be used for next operation
//                    if(middle_row==4)// if last row is used then come back to first row
//                        middle_row = 1;
//                    found_bin_idx = true;
//                }
//                else if(bin_drop_suggestion[obj_idx-1] == 0)
//                {
//                    if(!left_scanned)// if left column bins have not been stowd once
//                    {
//                        bin_idx = 3*left_row;
//                        n_obj_bin[bin_idx] -= 1;
//                        left_row++;
//                        if(left_row==4)
//                        {
//                            left_scanned = true;
//                            left_row = 1;
//                        }
//                        found_bin_idx = true;
//                    }
//                    else
//                    {
//                        bin_idx = 3*right_row+2;
//                        n_obj_bin[bin_idx] -= 1;
//                        right_row++;
//                        if(right_row==4)
//                        {
//                            left_scanned = false;
//                            right_row = 1;
//                        }
//                        found_bin_idx = true;

//                        // uncomment the following in case of not using right bin to stow
//                        //                        left_scanned = false;
//                    }
//                }
//                if(!found_bin_idx)
//                    bin_idx = 7;// In case it failed to generate a bin index
            }

            // copy the bin centroid and bin number to send to trajectory planning
            for(int j=0; j<3; j++)
                traj_stow.request.bin_centroid.data.push_back(centroidVector[bin_idx][j]);
            traj_stow.request.bin_num.data = bin_idx;


            if(trajStowService.call(traj_stow))
            {
                cout << "Obtained trajectory planning" << endl;
                success = true;
            }
            else
            {
                ROS_ERROR("Failed to call trajectory planning service");
                success = false;
            }
        }

        // execute the planned trajectory to reach and pick the object
        if(success)
        {
            // copy the trajectory joint angle values into the local variables
            int n_jts = 7;// Number of joints on arm
            int n_trj = traj_stow.response.jts_reach_tote.data.size()/n_jts;
            vector< vector<double> > traj_reach_obj = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

            n_trj = traj_stow.response.jts_back_tote.data.size()/n_jts;
            vector< vector<double> > traj_come_out = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

            n_trj = traj_stow.response.jts_reach_rack.data.size()/n_jts;
            vector< vector<double> > traj_rack = vector < vector<double> >(n_trj, vector<double>(n_jts,0));

            vector< vector<double> > traj_itd = vector < vector<double> >(num_itd, vector<double>(n_jts,0));

            cout << "Copy Trajectory angles to object in tote: " << traj_reach_obj.size() << "\n[";
            int r = 0,s=0;
            for(vector<double>::const_iterator it = traj_stow.response.jts_reach_tote.data.begin();
                it!=traj_stow.response.jts_reach_tote.data.end(); ++it)
            {
                traj_reach_obj[r][s] = *it;
                s++;
                if(s==n_jts)
                {
                    r++;
                    s=0;
                }
            }
            cout << "Copy Trajectory angles to come out of tote: " << traj_come_out.size() << "\n ";
            r = 0,s=0;
            for(vector<double>::const_iterator it = traj_stow.response.jts_back_tote.data.begin();
                it!=traj_stow.response.jts_back_tote.data.end(); ++it)
            {
                traj_come_out[r][s] = *it;
                s++;
                if(s==n_jts)
                {
                    r++;
                    s=0;
                }
            }
            cout << "Copy Trajectory to reach rack angles: " << traj_rack.size() << "\n ";
            r = 0,s=0;
            for(vector<double>::const_iterator it = traj_stow.response.jts_reach_rack.data.begin();
                it!=traj_stow.response.jts_reach_rack.data.end(); ++it)
            {
                traj_rack[r][s] = *it;
                s++;
                if(s==n_jts)
                {
                    r++;
                    s=0;
                }
            }

            // copy joint angles to go from tote to in front of rack
            for(int i=0; i<num_itd; i++)
            {
                for(int j=0; j<n_jts; j++)
                    traj_itd[i][j] = tote_to_rack[i][j];
            }

            for(int i=0; i<traj_reach_obj.size(); i++)
            {
                //                reach the object and turn on the vacuum cleaner
                char s[10];
                sprintf(s," %d", i);
                string str = "Reached traj position step:";
                str.append(s);

#if(COMMENT)
                cout << "Moving to: " << i << ": [";
                for(int j=0;j<7;j++)
                    cout << traj_reach_obj[i][j] << " ";
                cout << "]\n";
#endif
                jtMoveServiceCall(traj_reach_obj[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
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

            //            pick out the object from tote
            if(success)
            {
                for(int i=0; i<traj_come_out.size(); i++)
                {
                    char s[10];
                    sprintf(s," %d", i);
                    string str = "Coming out traj position step:";
                    str.append(s);
#if(COMMENT)
                    cout << "Moving to: " << i << ": [";
                    for(int j=0;j<7;j++)
                        cout << traj_come_out[i][j] << " ";
                    cout << "]\n";
#endif
                    jtMoveServiceCall(traj_come_out[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
                }
            }

            // put IR checking whether object is there or not
            // go further only if object is still there
            // else turn on the vacuum cleaner and make success = false
            // check whether object is still there using ir sensor data
            while(!ir_data_available)
                ros::spinOnce();
            ir_data_available = false;


            // for objects which are in ir checking exception list consider object is there
            int object_index = stow_obj_detect.response.recognized_obj_id.data;
            if(ir_exception_list[object_index-1] == 1)
                ir_val = IR_SENSOR_VAL+100;

            // check whether ir_val is less than 1000
            if(success && ir_val < IR_SENSOR_VAL) // if ir value is less than 1000 then object is not there, go to next trail
            {
                success = false;

                // turn off vacuum cleaner
                ros::Rate loop_rate(10);
                std_msgs::Int16 vaccumData;
                vaccumData.data = 2;
                for(int i=0;i<10;i++)
                {
                    vaccumCleanerPub.publish(vaccumData);
                    loop_rate.sleep();
                }
                //                sleep(1);
                retry_recog = true;

                cout << "No object is held by suction\nTurned off vaccum cleaner" << endl;
            }
            else
                retry_recog = false;

            // go infront of bin
            if(success)
            {
                for(int i=0; i<traj_itd.size(); i++)
                {
                    char s[10];
                    sprintf(s," %d", i);
                    string str = "Going in front of rack step:";
                    str.append(s);
#if(COMMENT)
                    cout << "Moving to: " << i << ": [";
                    for(int j=0;j<7;j++)
                        cout << traj_itd[i][j] << " ";
                    cout << "]\n";
#endif

                    jtMoveServiceCall(traj_itd[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
                }
                // put IR checking whether object is there or not
                // go further only if object is still there
                {

                }

                if(success)
                {
                    cout << "\nNumber trajectory rack inside: " << traj_rack.size() << endl;
                    for(int i=0; i<traj_rack.size(); i++)
                    {
                        char s[10];
                        sprintf(s," %d", i);
                        string str = "Going inside rack step:";
                        str.append(s);
#if(COMMENT)
                        cout << "Moving to: " << i << ": [";
                        for(int j=0;j<7;j++)
                            cout << traj_rack[i][j] << " ";
                        cout << "]\n";
#endif

                        //                        char c;
                        //                        cout << "Press c" << endl;
                        //                        cin >> c;
                        //                        getchar();

                        jtMoveServiceCall(traj_rack[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
                    }
                }
            }

            // Turn off the vaccum cleaner
#if(VACCUM_SUCTION)
            if(success)
            {
                ros::Rate loop_rate(10);
                std_msgs::Int16 vaccumData;
                vaccumData.data = 2;
                for(int i=0;i<10;i++)
                {
                    vaccumCleanerPub.publish(vaccumData);
                    loop_rate.sleep();
                }
                sleep(3);
                success = true;
                cout << "Turned off vaccum cleaner" << endl;
            }
#endif

            if(success)
            {
                // update the object's location in the json file
                for(int i=0; i<tote_obj.size(); i++)
                {
                    if(tote_obj[i] == stow_obj_detect.response.recognized_obj_id.data)
                    {
                        tote_obj_picked[i] = true;
                        cout << model_names[tote_obj[i]-1] << " is picked and put into " << bin_names[bin_idx] << endl;
                        apc_controller::write_stow_data write_stow_data;
                        write_stow_data.request.bin_id.data = bin_idx;
                        write_stow_data.request.obj_id.data = tote_obj[i];

                        if(write_stow_data_service.call(write_stow_data))
                        {
                            cout << "Wrote into file " << tote_obj[i] << ": " << model_names[tote_obj[i]-1]
                                 << " is put into " << bin_names[bin_idx] << endl;
                            success = true;
                        }
                        else
                            cout << "Failed to call write tote object data service" << endl;

                        break;
                    }
                }

                // update bin blacklist status
                current_bin_items_number[bin_idx] += 1;
                if(current_bin_items_number[bin_idx] > MAX_BIN_CONTENTS)
                    bin_blacklist_status[bin_idx] = true;

                // come out of the rack
                for(int i=traj_rack.size()-2; i>=0; i--)
                {
                    char s[10];
                    sprintf(s," %d", i);
                    string str = "Come out of rack step:";
                    str.append(s);
#if(COMMENT)
                    cout << "Moving to: " << i << ": [";
                    for(int j=0;j<7;j++)
                        cout << traj_rack[i][j] << " ";
                    cout << "]\n";
#endif

                    //                    char c;
                    //                    cout << "Press c" << endl;
                    //                    cin >> c;
                    //                    getchar();

                    jtMoveServiceCall(traj_rack[i], simJointPublisher, poseService, wam_current_jts, str.c_str());
                }
            }
        }

        // check whether all pickable have been picked or not
        stow_over = true;
        for(int i=0; i<tote_obj_picked.size(); i++)
        {
            if(!tote_obj_picked[i])
            {
                stow_over = false;
                success = true;
                break;
            }
        }

//        char c;
//        cout << "Check the object picking ..." << endl;
//        cin >> c;
//        getchar();

    }


    cout << "Completed stow task successfully :)" << endl;
    jtMoveServiceCall(start_int_pose, simJointPublisher, poseService, wam_current_jts, "Start intermediate position reached ");
    jtMoveServiceCall(home_pose, simJointPublisher, poseService, wam_current_jts, "Start position reached ");
}
