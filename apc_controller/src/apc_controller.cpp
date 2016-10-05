#include <iostream>

#include <apc_controller.h>


void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

//    sleep(1);
//    exit(signum);
    exit(0);
}

#define COMMENT 0

int main(int argc,char **argv)
{
    ros::init(argc,argv,"apc_controller");

    ros::NodeHandle nh;

    ros::ServiceClient shelfService = nh.serviceClient<lines_rack_det::DetectShelf>("/detect_shelf");
    ros::ServiceServer shelf_dataService = nh.advertiseService("/shelf_data", shelfDataCallback);
    ros::ServiceClient linesRackDetectService = nh.serviceClient<lines_rack_det::DetectShelf>("/lines_rack_detect");

    ros::ServiceClient ikService = nh.serviceClient<wam_ikfast_7dof_service::PoseJoint7dof>("/ik_centre_joints");
    ros::ServiceClient ik_kinectService = nh.serviceClient<wam_ikfast_7dof_service::PoseJoint7dof>("/ik_kinect_view_joints");

    ros::Publisher simJointPublisher = nh.advertise<sensor_msgs::JointState>("/apc_controller/joint_states",1);

//    sensor_msgs::PointCloud2::Ptr pc_msg(new sensor_msgs::PointCloud2 ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
    bool get_cloud = true;
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1,
                                                                         boost::bind(kinectPCCallback, _1, boost::ref(cloud), boost::ref(get_cloud)));

    vector<double> wam_current_jts(7);
    vector<double> wam_desired_jts(7);

    ros::Subscriber sub_wam_joints = nh.subscribe<sensor_msgs::JointState>
            ("/wam/joint_states",1,boost::bind(wamJointStateCallback,_1,boost::ref(wam_current_jts)));

#if(SHOW_SIMULATION_MARKER)
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/eef_position",10);
#endif

#if(ROBOT)
    ros::ServiceClient poseService = nh.serviceClient<apc_controller::JointMove>("/wam/joint_move");
    apc_controller::JointMove joint_srv_start,joint_srv_home, joint_srv_start_int;
    apc_controller::JointMove joint_srv_toteC1, joint_srv_toteC2, joint_srv_toteC3;
#endif


#if(VACCUM_SUCTION)
    ros::Publisher vaccumCleanerPub = nh.advertise<std_msgs::Int16>("/vaccum_control",1);
#endif


#if(STOW_TASK)

    ros::ServiceClient pick_obj_service = nh.serviceClient<json_maker::get_bin_object>("/pick_object_service");
    ros::ServiceClient pick_status_write_service = nh.serviceClient<json_maker::write_pick_status>("/pick_object_status_service");
    ros::ServiceClient write_stow_data_service = nh.serviceClient<json_maker::write_stow_data>("/write_stow_data_service");

    double above_tote_angles[7] = {-1.631, 0.153, 0.117, 2.129, -0.058, 0.713, -0.101};
    double stow_view[7] = {-1.484, 0.462, -0.097, 2.539, 1.220, 1.012, -1.436};
    double inter_bin_angles[7] = {0.0, -0.9, 0.0, 2.3, 0.0, 0.3, 0.0};//{0.0, 0.0, 0.0, 1.25, 0.0, 0.0, 0.0}



    apc_controller::JointMove stow_above_tote_jt_angles, stow_obj_view_jt_angles, stow_inter_bin_angles;
    for(int i=0;i<7;i++)
    {
//        double temp1=0.0,temp2=0.0;
//        above_toteFile  >> temp1;
//        stow_obj_viewFile >> temp2;
//        stow_above_tote_jt_angles.request.joints.push_back(temp1);
//        stow_obj_view_jt_angles.request.joints.push_back(temp2);

        stow_above_tote_jt_angles.request.joints.push_back(above_tote_angles[i]);
        stow_obj_view_jt_angles.request.joints.push_back(stow_view[i]);
        stow_inter_bin_angles.request.joints.push_back(inter_bin_angles[i]);
    }
#if(DEFAULT_TOTE_OBJ_POSE)
    double tote_obj_posn[3][3] = {{-0.15,0.05,0.68},{0.1,0.05,0.725},{-0.28,0.05,0.68}};
    int stow_count = 0;
#endif

#endif

#if(WEBCAM_TOTE_VIDEO_RECORD)
    ros::Publisher tote_webcam_record_pub = nh.advertise<std_msgs::Bool>("webcam_record/tote",5);
    std_msgs::Bool tote_webcam_record_flag;
#endif
    // Start Position
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

#if(ROBOT)


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
#endif


    /// Object bin related file data
#if(OBJECT_BIN)
    ifstream object_bin_upFile,object_bin_downFile;
    std::string dataPath_up = ros::package::getPath("apc_controller").append("/data/object_bin_up.txt");
    std::string dataPath_down = ros::package::getPath("apc_controller").append("/data/object_bin_down.txt");

    object_bin_upFile.open(dataPath_up.c_str());
    object_bin_downFile.open(dataPath_down.c_str());

    if(!object_bin_upFile.is_open())
    {
        cerr << "Error opening object bin up file" << endl;
        exit(0);
    }

    if(!object_bin_downFile.is_open())
    {
        cerr << "Error opening object bin down file" << endl;
        exit(0);
    }

    apc_controller::JointMove object_bin_up,object_bin_down;
    for(int i=0;i<7;i++)
    {
        double temp1=0.0,temp2=0.0;
        object_bin_upFile  >> temp1;
        object_bin_downFile >> temp2;
        object_bin_up.request.joints.push_back(temp1);
        object_bin_down.request.joints.push_back(temp2);
    }



#endif




#if(ROBOT_ANGLE_SIMULATION)

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

    vector<double> object_bin_up_sim;
    vector<double> object_bin_down_sim;
    object_bin_upFile.seekg(0,std::ios_base::beg);
    object_bin_downFile.seekg(0,std::ios_base::beg);

    for(int i=0;i<7;i++)
    {
        double temp =  0.0;
        object_bin_upFile >> temp;
        object_bin_up_sim.push_back(temp);
    }

    for(int i=0;i<7;i++)
    {
        double temp =  0.0;
        object_bin_downFile >> temp;
        object_bin_down_sim.push_back(temp);
    }

#endif

    // Shelf detection variables
    lines_rack_det::DetectShelf dhselfCentroid;
    vector<double> centroidArray;
    int shelfBinIndex = 0;

    wam_ikfast_7dof_service::PoseJoint7dof poseJoint;

#if(ROBOT)
    // Shelf bin pose variable
    apc_controller::JointMove joint_srv_shelf_bin;
#endif

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


#if(RACK_REGISTRATION)
    // rack registration service to register rack as per ashish
    ros::ServiceClient rackRegistrationService = nh.serviceClient<apc_controller::rack_registration>("/rack_registration");
#endif
    while(ros::ok())
    {
        int input = 0;
        int count = 0;
        signal(SIGINT, signal_callback_handler);


        cout << "\n Press 0 for Start Position "
             << "\n Press 1 for Home Position "
             << "\n Press 3 to reach bin with tip "
             << "\n Press 11 for lines based rack corner detection"
             << "\n Press 12 for moving robot fwd or bwd in multiples of +-0.245"
             << "\n Press 13 for moving robot up +-0.3m"
             << "\n Press 14 for Rack Pick object detection and planning"
             << "\n Press 15 displays wam base link to camera depth optical link frame transform"
             << "\n Press 16 to go to start position"
             << "\n Press 17 to go in front of bin4 to check calibration"
             << "\n Press 19 to call rack registration service"
             << endl;
        cin >> input;
        getchar();

        switch(input)
        {
        case 0:
            cout << "Start position call" << endl;

#if(ROBOT)
#if(ROBOT_ANGLE_SIMULATION)
            publishJointStateInSimulation(start_int_pose,simJointPublisher);
#endif
            if(poseService.call(joint_srv_start_int))
            {
#if(COMMENT)
                cout << "Start intermediate position reached " << endl;
#else
                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start_int.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    cout << "Start intermediate position reached " << endl;
                else
                    cout << "Failed to reach desired wam position" << endl;
#endif
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }

#if(ROBOT_ANGLE_SIMULATION)
            publishJointStateInSimulation(start_pose,simJointPublisher);
#endif
            if(poseService.call(joint_srv_start))
            {
#if(COMMENT)
                cout << "Start position reached " << endl;
#else
                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    cout << "Start position reached " << endl;
                else
                    cout << "Failed to reach desired wam position" << endl;
#endif
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
#endif


            break;
        case 1:
            cout << "Home position call" << endl;

#if(ROBOT)
#if(ROBOT_ANGLE_SIMULATION)
            publishJointStateInSimulation(start_int_pose,simJointPublisher);
#endif
            if(poseService.call(joint_srv_start_int))
            {
#if(COMMENT)
                cout << "Start intermediate position reached " << endl;
#else
                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start_int.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    cout << "Start intermediate position reached " << endl;
                else
                    cout << "Failed to reach desired wam position" << endl;
#endif
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }

#if(ROBOT_ANGLE_SIMULATION)
            publishJointStateInSimulation(home_pose,simJointPublisher);
#endif
            if(poseService.call(joint_srv_home))
            {
#if(COMMENT)
                cout << "Home position reached " << endl;
#else
                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_home.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    cout << "Home position reached " << endl;
                else
                    cout << "Failed to reach desired wam position" << endl;
#endif
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
#endif


            break;
            // Case for particular shelf detection
        case 3:
        {
            ROS_INFO("Press bin index between 0-11");
            cin >> shelfBinIndex;
//            preTfPoseJoint.request.pose.position.x = centroidVector[shelfBinIndex][0];
//            preTfPoseJoint.request.pose.position.y = centroidVector[shelfBinIndex][1];
//            preTfPoseJoint.request.pose.position.z = centroidVector[shelfBinIndex][2];


            // Call service for kinect to robot transformaton of centroids
//            if(tf_kinectService.call(preTfPoseJoint))
//            {
//                poseJoint.request.pose.position.x = preTfPoseJoint.response.pose.position.x;
//                poseJoint.request.pose.position.y = preTfPoseJoint.response.pose.position.y;
//                poseJoint.request.pose.position.z = preTfPoseJoint.response.pose.position.z;

                poseJoint.request.pose.position.x = centroidVector[shelfBinIndex][0];
                poseJoint.request.pose.position.y = centroidVector[shelfBinIndex][1];
                poseJoint.request.pose.position.z = centroidVector[shelfBinIndex][2];

                poseJoint.request.bin_num.data = shelfBinIndex;

                std::cout << "Centroid \t" <<  poseJoint.request.pose.position.x << "\t" << poseJoint.request.pose.position.y << "\t" <<
                             poseJoint.request.pose.position.z << "\t" << poseJoint.request.bin_num.data << endl;

                if(ikService.call(poseJoint))
                {
                    cout << " Bin service call done " << endl;
                    //                    std_msgs::Int16 send_bin_num;
                    //                    send_bin_num.data = shelfBinIndex;
                    //                    ros::Rate loop_rate(10);
                    //                    for(int i = 0; i < 10; i++)
                    //                    {
                    //                        bin_num_pub.publish(send_bin_num);
                    //                        loop_rate.sleep();
                    //                    }
#if(SHOW_SIMULATION_MARKER)

//                    IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*7);
                    IkReal eerot[9],eetrans[3];
//                    for(int i = 0; i < binPose.size(); i++)
//                        jts[i] = binPose[i];
//                    ComputeFk(jts, eetrans, eerot);

                    vector<double> centroid_kinect,centroid_wam, centroid_fk;
                    centroid_kinect.push_back(centroidVector[shelfBinIndex][0]);
                    centroid_kinect.push_back(centroidVector[shelfBinIndex][1]);
                    centroid_kinect.push_back(centroidVector[shelfBinIndex][2]);

                    centroid_wam.push_back(centroidVector[shelfBinIndex][0]);
                    centroid_wam.push_back(centroidVector[shelfBinIndex][1]);
                    centroid_wam.push_back(centroidVector[shelfBinIndex][2]);

                    centroid_fk.push_back(eetrans[0]);
                    centroid_fk.push_back(eetrans[1]);
                    centroid_fk.push_back(eetrans[2]);
                    showMarkerInSimulation(centroid_kinect,centroid_wam,centroid_fk,marker_array_pub);
#endif

#if(ROBOT_ANGLE_SIMULATION)
                    vector<double> binPose;
#endif


#if(ROBOT)
                    joint_srv_shelf_bin.request.joints.clear();
                    for(vector<double>::const_iterator it= poseJoint.response.joint_angles.data.begin();
                        it!=poseJoint.response.joint_angles.data.end();++it)
                    {
                        joint_srv_shelf_bin.request.joints.push_back(*it);
#if(ROBOT_ANGLE_SIMULATION)
                        binPose.push_back(*it);
#endif
                    }

                    if(poseService.call(joint_srv_shelf_bin))
                    {
                        cout << "Service call done: Robot reaching shelf bin outside " << endl;
                    }
                    else
                    {
                        cout << "Service failed" << endl;
                    }
#endif
#if(ROBOT_ANGLE_SIMULATION)
                    publishJointStateInSimulation(binPose,simJointPublisher);
#endif

                }
                else
                {
                    cout << "IK Service not reachable" << endl;
                }
//            }
//            else
//            {
//                cout << "tf kinect robot service not reachable" << endl;
//            }
    }
            break;

        case 11:
        {
            bool success = false;

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

                count = 0;
                if(centroidArray.size() > 0)
                {
//                    cout << "Bin centroids: \n";
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



            break;
        }

        case 12:
        {
            // for moving robot fwd 0.3m
            double d;
            cout << "Enter the distance to move fwd or bwd" << endl;
            cout << "+0.27 +0.54 or -0.27 -0.54" << endl;
            cin >> d;
            xwam_fwdback.request.distance.data = d;
            if(motionFwdBckService.call(xwam_fwdback))
            {
                cout << "Successful forward motion" << endl;
                xwam_dist_err = xwam_fwdback.response.error.data;
                d -= xwam_dist_err;
                cout << "Robot moved in x: " << d << endl;
                translateCentroidCorners(d,0.0,0.0);
            }
            else
                cout << "Failed to call xwam forward motion service" << endl;
            break;
        }

        case 13:
        {
            // for moving robot up 0.2m
            double h;
            cout << "Enter the height +0.15 or -0.15 to go up or dwn: " ;
            cin >> h;
            getchar();
            xwam_updown.request.height.data = h;
            if(motionUpDownService.call(xwam_updown))
            {
                cout << "Successful upward motion" << endl;
                h += xwam_updown.response.error.data;
                cout << "Robot moved in z: " << h << endl;
                translateCentroidCorners(0.0,0.0,h);
            }
            else
                cout << "Failed to call xwam upward motion service" << endl;
            break;
        }

        case 14:
        {
            bool success = true;
            int bin_idx;
            char c;

            cout << "Enter the bin number for bin view: ";
            cin >> bin_idx;
            getchar();

            // Call service to get joint angles for bin view position
            if(success)
            {
//                poseJoint.request.pose.position.x = centroidVector[shelfBinIndex][0];
//                poseJoint.request.pose.position.y = centroidVector[shelfBinIndex][1];
//                poseJoint.request.pose.position.z = centroidVector[shelfBinIndex][2];

//                poseJoint.request.bin_num.data = bin_idx;

//                // call the service
//                if(ik_kinectService.call(poseJoint))
//                {
//                    cout << "service done: bin view joint angle" << endl;
//                    success = true;
//                }
//                else
//                {
//                    cout << "service not done: bin view joint angle" << endl;
//                    success = false;
//                }
            }

            apc_controller::JointMove jt_move;
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
            // call service to send arm to bin view position
            if(success)
            {
                jt_move.request.joints.clear();
//                for(vector<double>::const_iterator it = poseJoint.response.joint_angles.data.begin();
//                    it!=poseJoint.response.joint_angles.data.end();++it)
//                    jt_move.request.joints.push_back(*it);
                for(int j=0; j<7; j++)
                    jt_move.request.joints.push_back(bin_angles[bin_idx][j]);

#if(ROBOT_ANGLE_SIMULATION)
            vector<double> binPose(7);
            for(int j=0;j<7;j++)
                binPose[j] = jt_move.request.joints[j];
            publishJointStateInSimulation(binPose,simJointPublisher);
#endif
                if(poseService.call(jt_move))// call the service
                {
                    for(int i = 0; i < wam_desired_jts.size(); i++)
                        wam_desired_jts[i] = jt_move.request.joints[i];
                    if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    {
                        cout << "service done: arm reached bin view " << endl;
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
                    cout << "service not done: arm reached bin view " << endl;
                    success = false;
                }
            }

            // *********************************
            // Display bin corners after transformation
            {
                tf::StampedTransform WAMtoKttransform;
                getWAMBaseToKinect(WAMtoKttransform);
                vector<tf::Vector3> tf_corners(bin_corners.size());
                for(int j=0; j<bin_corners.size(); j++)
                {
                    tf_corners[j] = WAMtoKttransform*bin_corners[j];
                }

                sphereMarker(tf_corners, marker_array_pub);
            }

            int detection_method, obj_id_rpt;
            string methods[] = {"RCNN"};
            sleep(2);// sleep for 3s so that arm reaches it position
            if(success)
//            if(0)
            {
                cout << "Enter the object id to be detected: " ;
                cin >> obj_id_rpt;
                getchar();
//                obj_id_rpt = ;
                cout << "Enter the object detection method to be used: ";
//                cin >> detection_method;
//                getchar();
                detection_method = 1; cout << "RCNN\n";
            }


            geometry_msgs::Point obj_centroid_rpt, obj_normal_rpt, obj_left, obj_top, obj_right;

            // Rack pick object detection and planning
            if(success)
//            if(0)
            {

                get_cloud = true;
                while(get_cloud && ros::ok())
                {
                    get_cloud = true;
                    ros::spinOnce();
                }
                cout << "Cloud found" << endl;

                if(detection_method == 1)//If object detection method 1 is to be used
                {
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

//                    for(int j=0; j<4; j++)
//                    {
//                        tf::Vector3 corner = bin_corners[4*bin_idx+j];
//                        tf::Vector3 tf_corner = WAMtoKttransform*corner;
//                        obj_detect_rcnn.request.bin_corners.data.push_back(tf_corner.getX());
//                        obj_detect_rcnn.request.bin_corners.data.push_back(tf_corner.getY());
//                        obj_detect_rcnn.request.bin_corners.data.push_back(tf_corner.getZ());
//                    }
                    cout << "Called RCNN object detection service" << endl;
                    if(objDetectRCNNService.call(obj_detect_rcnn))//call object detection service method 1
                    {
                        cout << "service done: object detection obtained by method " << methods[detection_method-1] << endl;
                        // copy object centroid and object normal to local variable
                        kinectToWAMBase(obj_detect_rcnn.response.centroid, obj_centroid_rpt);
                        kinectToWAMBase(obj_detect_rcnn.response.obj_left, obj_left);
                        kinectToWAMBase(obj_detect_rcnn.response.obj_top, obj_top);
                        kinectToWAMBase(obj_detect_rcnn.response.obj_right, obj_right);
                        kinectToWAMVector(obj_detect_rcnn.response.normal, obj_normal_rpt);
                        success = true;
                    }
                    else
                    {
                        cout << "service not done: object detection obtained by method " << detection_method << endl;
                        success = false;
                    }
                }
                else if(detection_method == 2)//If object detection method 2 is to be used
                {
                    if(1)//call object detection service method 1
                    {
                        cout << "service done: object detection obtained by method " << detection_method << endl;
                        // copy object centroid and object normal to local variable
//                    obj_centroid_rpt;
//                    obj_normal_rpt;
                        success = true;
                    }
                    else
                    {
                        cout << "service not done: object detection obtained by method " << detection_method << endl;
                        success = false;
                    }
                }
            }

            success = false; // temporarily not going for trajectory planning
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

//                        if(trajRpt.response.idx_bend.data.size()>0)
//                        {
//                            if(i==trajRpt.response.idx_bend.data[0])
//                            {
//                                // call service to bend the pipe at eef
//                                if(1)
//                                {
//                                    cout << "service done: Tool eef bent" << endl;
//                                    success = true;
//                                }
//                                else
//                                {
//                                    cout << "service not done: Tool eef not bent" << endl;
//                                    success = false;
//                                }
//                            }
//                        }

#if(ROBOT_ANGLE_SIMULATION)
                        vector<double> binPose(7);
                        for(int j=0;j<7;j++)
                            binPose[j] = jt_move.request.joints[j];
                        publishJointStateInSimulation(binPose,simJointPublisher);
#endif

//                        cout << "Moving to: " << i << ": [";
//                        for(int j=0;j<7;j++)
//                            cout << jt_move.request.joints[j] << " ";
//                        cout << "]\n";

//#if(SHOW_SIMULATION_MARKER)

//                        {
//                            IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*7);
//                            IkReal eerot[9],eetrans[3];
//                            for(int j=0;j<7;j++)
//                                jts[i] = jt_move.request.joints[j];

//                            ComputeFk(jts, eetrans, eerot);

//                            vector<double> centroid_kinect(3),centroid_wam, centroid_fk;

//                            centroid_wam.push_back(centroidVector[bin_idx][0]);
//                            centroid_wam.push_back(centroidVector[bin_idx][1]);
//                            centroid_wam.push_back(centroidVector[bin_idx][2]+0.46);

//                            wamBaseToKinect(centroid_wam,centroid_kinect);
//                            centroid_fk.push_back(eetrans[0] - 0.22);
//                            centroid_fk.push_back(eetrans[1] - 0.14);
//                            centroid_fk.push_back(eetrans[2] - 0.406+0.46);
//                            showMarkerInSimulation(centroid_kinect,centroid_wam,centroid_fk,marker_array_pub);
//                        }
//#endif


                        cout << "Moving to: " << i << ": [";
                        for(int j=0;j<7;j++)
                            cout << jt_move.request.joints[j] << " ";
                        cout << "]\n";
                        if(i==0 || i==1)
                        {
                            cout << "Press 'c': " ;
                            cin >> c;
                            getchar();
                        }
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
                            break;
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
                    cout << "Turned off vaccum cleaner" << endl;
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

//                            if(trajRpt.response.idx_bend.data.size()>1)
//                            {
//                                if(i==trajRpt.response.idx_bend.data[1])
//                                {
//                                    // call service to bend the pipe at eef
//                                    if(1)
//                                    {
//                                        cout << "service done: Tool eef bent" << endl;
//                                        success = true;
//                                    }
//                                    else
//                                    {
//                                        cout << "service not done: Tool eef not bent" << endl;
//                                        success = false;
//                                    }
//                                }
//                            }

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
//                            cout << "Press 'c': " ;
//                            cin >> c;
//                            getchar();

//#if(SHOW_SIMULATION_MARKER)

//                        {
//                            IkReal *jts = (IkReal*) malloc(sizeof(IkReal)*7);
//                            IkReal eerot[9],eetrans[3];
//                            for(int j=0;j<7;j++)
//                                jts[i] = jt_move.request.joints[j];
//                            ComputeFk(jts, eetrans, eerot);

//                            vector<double> centroid_kinect(3),centroid_wam, centroid_fk;

//                            centroid_wam.push_back(centroidVector[bin_idx][0]);
//                            centroid_wam.push_back(centroidVector[bin_idx][1]);
//                            centroid_wam.push_back(centroidVector[bin_idx][2]);

//                            wamBaseToKinect(centroid_wam,centroid_kinect);
//                            centroid_fk.push_back(eetrans[0]);
//                            centroid_fk.push_back(eetrans[1]);
//                            centroid_fk.push_back(eetrans[2]);
//                            showMarkerInSimulation(centroid_kinect,centroid_wam,centroid_fk,marker_array_pub);
//                        }
//#endif
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

                if(success)
                {
                    // Go to tote drop position
#if(ROBOT)
                    apc_controller::JointMove joint_move;
                    vector<double> tote_pose;
                    if(bin_idx==0 || bin_idx == 3 || bin_idx==6 || bin_idx==9)
                    {
                        for(int i=0; i<7; i++)
                        {
                            joint_move.request.joints.push_back(toteC1_pose[i]);
                            tote_pose.push_back(toteC1_pose[i]);
                        }
                    }
                    else if(bin_idx==1 || bin_idx == 4 || bin_idx==7 || bin_idx==10)
                    {
                        for(int i=0; i<7; i++)
                        {
                            joint_move.request.joints.push_back(toteC2_pose[i]);
                            tote_pose.push_back(toteC2_pose[i]);
                        }
                    }
                    else
                    {
                        for(int i=0; i<7; i++)
                        {
                            joint_move.request.joints.push_back(toteC3_pose[i]);
                            tote_pose.push_back(toteC3_pose[i]);
                        }
                    }
#if(ROBOT_ANGLE_SIMULATION)
                    publishJointStateInSimulation(tote_pose,simJointPublisher);
#endif
                    success = false;
                    if(poseService.call(joint_move))
                    {
                        cout << "tote position reached " << endl;

                        for(int i = 0; i < wam_desired_jts.size(); i++)
                            wam_desired_jts[i] = joint_move.request.joints[i];
                        if(checkJointsReached(wam_desired_jts,wam_current_jts))
                            cout << "tote position reached " << endl;
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
                if(success)
                {
// Turn off the vaccum cleaner
#if(VACCUM_SUCTION)
                    ros::Rate loop_rate(10);
                    std_msgs::Int16 vaccumData;
                    vaccumData.data = 0;
                    for(int i=0;i<10;i++)
                    {
                        vaccumCleanerPub.publish(vaccumData);
                        loop_rate.sleep();
                    }
                    success = true;
                    cout << "Turned off vaccum cleaner" << endl;
#endif
                }
            }

            cout << "Arm has picked the object and come outside" << endl;
            break;
        }

        case 15:
        {
            wamBasetoDepthFrameTr();
            break;
        }

        case 16:
        {
#if(ROBOT)
//#if(ROBOT_ANGLE_SIMULATION)
//            publishJointStateInSimulation(start_int_pose,simJointPublisher);
//#endif

#if(ROBOT_ANGLE_SIMULATION)
            publishJointStateInSimulation(start_pose,simJointPublisher);
#endif
#if(COMMENT)
            if(1)
#else
            if(poseService.call(joint_srv_start))
#endif
            {
#if(COMMENT)
                cout << "Start position reached " << endl;
#else
                for(int i = 0; i < wam_desired_jts.size(); i++)
                    wam_desired_jts[i] = joint_srv_start.request.joints[i];
                if(checkJointsReached(wam_desired_jts,wam_current_jts))
                    cout << "Start position reached " << endl;
                else
                    cout << "Failed to reach desired wam position" << endl;
#endif
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
#endif
            break;
        }

        case 19:
        {
#if(RACK_REGISTRATION)

            get_cloud = true;
            while(get_cloud && ros::ok())
            {
                get_cloud = true;
                ros::spinOnce();
            }
            cout << "Cloud found\nCaution: Arm should be in start position" << endl;
            bool success = true;
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
                success = true;
            }
            else
            {
                cout << "Failed to call service for rack registration" << endl;
                success = false;
            }
#endif
            break;
        }

        default:
            cout << "nothing matched : default case" << endl;
            break;

        }


    }



    return 0;
}



