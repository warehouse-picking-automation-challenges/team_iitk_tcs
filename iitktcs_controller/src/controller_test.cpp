#include <ros/ros.h>
#include <iitktcs_controller/controller_test_utils.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"controller_test");
    ros::NodeHandle nh;

    cout << "In controller test program" << endl;
    ros::ServiceClient client_single_goal =
            nh.serviceClient<iitktcs_msgs_srvs::UR5Goal>("/iitktcs/motion_planner/single_goal");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_bbox(new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ());
    cv::Mat src_image, image_bbox;

    ros::Subscriber sub_image =
            nh.subscribe<sensor_msgs::Image>
            ("/iitktcs/foscam/image_color",1,boost::bind(imageCallback,_1,boost::ref(src_image)));
    ros::Subscriber sub_cloud =
            nh.subscribe<sensor_msgs::PointCloud2>
            ("/iitktcs/ensenso/depth_registered/points", 1,
             boost::bind(kinectPCCallback, _1, boost::ref(src_cloud)));

    //*********** Ensenso Foscam stuff **********//
    ros::ServiceClient client_ensenso_registered_cloud =
            nh.serviceClient<iitktcs_msgs_srvs::EnsensoRegisteredCloud>("/iitktcs/ensenso/registered_cloud");

    double bin_jts[][NO_JOINTS] = {{-0.0109823, -1.56805, 1.37091, -1.11577, -1.58023, 0.0018583},
                                   {-0.570878, -1.5398, 1.35223, -1.16178, -1.71734, -0.532907},
                                   {1.99343, -1.3337, 1.38803, -1.34644, -1.43215, 0.337836},// press 2 left pack 0 view
                                   {1.95587, -1.00293, 1.63985, -2.13496, -1.57917, 0.292248}, //press 3 left pack 0 drop
                                   {1.45534, -1.58904, 1.66518, -1.38629, -1.59178, -0.177695}, // press 4 left pack 1 view
                                   {1.48507, -1.19148, 1.93048, -2.27077, -1.59004, -0.156487}, //press 5 left pack 1 drop
                                   {0.880835, -1.46296, 1.54236, -1.43419, -1.73241, -0.697552}, // press 6 left pack 2 view
                                   {0.954183, -1.11725, 1.84674, -2.31828, -1.6072, -0.717242}, // press 7 left pack 2 drop
                                   {-2.17591, -1.49156, 1.54613, -1.3822, -1.75844, -0.567945}, // press 8 right pack 3 view
                                   {-2.19653, -1.08382, 1.87149, -2.32378, -1.51144, -0.607985},// press 9 right pack 3 drop
                                   {-1.37183, -1.46513, 1.50598, -1.33244, -1.54755, 0.212478},// press 10 right pack 4 view
                                   {-1.42717, -1.13493, 1.78685, -2.19087, -1.54918, 0.153773},// press 11 right pack 4 drop
                                   {1.2563183307647705, -1.5460737387286585, 1.5444064140319824, -1.3489583174334925, -1.6465542952166956, -0.3038438002215784} // press 12 for ashish jts angles
                                   };
//    double bin_jts[][NO_JOINTS] = {{1.84853, -1.38824, 2.17271, -2.24434, -1.51513, 0.243446}};
    int id_object;
    geometry_msgs::Pose target_object_collision_pose;
    geometry_msgs::Point target_object_axis, target_object_normal, target_object_centroid;

    ros::ServiceClient client_object_pose_estimate =
            nh.serviceClient<iitktcs_msgs_srvs::pose>("/iitktcs/estimate_pose");
    ros::ServiceClient client_suction_pose_goal =
            nh.serviceClient<iitktcs_msgs_srvs::UR5PoseGoal>("/iitktcs/motion_planner/suction_pose_goal");
    ros::ServiceClient client_object_detection =
            nh.serviceClient<iitktcs_msgs_srvs::computer_vision_stowing_picking>("/iitktcs/computer_vision_picking");
    ros::ServiceClient client_forward_kinematics =
            nh.serviceClient<iitktcs_msgs_srvs::GetFK>("/iitktcs/motion_planner/forward_kinematics");
    ros::ServiceClient client_simple_pose_goal =
            nh.serviceClient<iitktcs_msgs_srvs::UR5PoseGoal>("/iitktcs/motion_planner/simple_pose_goal");

    sensor_msgs::PointCloud2 cloud_object_roi;
    Eigen::Affine3d suction_eef, gripper_eef;
    int count = 0;
    while(ros::ok())
    {
        cout << "Press 2: goes to bin_view jts" << endl;
        cout << "Press 3: Capture  cloud and image from ensenso" << endl;
        cout << "Press 4: Get bbox for the target object" << endl;
        cout << "Press 5: Get target object pose using 3d pose matching" << endl;
        cout << "Press 6: Call suction pose goal" << endl;
        cout << "Press 7: Call object detection service" << endl;
        cout << "Press 8: Get forward kinematics for suction" << endl;
        cout << "Press 9: Get forward kinematics for gripper" << endl;
        cout << "Press 10: Go for simple pose goal" << endl;

        int input;
        cin >> input;
        getchar();

        switch(input)
        {
        case 2:
        {
            iitktcs_msgs_srvs::UR5Goal goal;

            cout << "Press bin index: ";
            int bin_num;
            cin >> bin_num;

            for(int j=0; j<NO_JOINTS; j++)
                goal.request.goal.data.push_back(bin_jts[bin_num][j]);

            if(client_single_goal.call(goal))
                cout << "Moved to target position" << endl;
            else
                cout << "Failed to move to target position" << endl;

            break;
        }
        case 3:
        {
            iitktcs_msgs_srvs::EnsensoRegisteredCloud ensenso_registered_cloud;

            cv::namedWindow("Calibrated_foscam_image", cv::WINDOW_NORMAL);

            cout << "Capturing cloud and image from ensenso" << endl;

            bool got_cloud = false;
            while(ros::ok() && !got_cloud)
            {
                ensenso_registered_cloud.request.get_cloud.data = true;

                if(client_ensenso_registered_cloud.call(ensenso_registered_cloud))
                {
                    while(ros::ok() && src_image.empty())
                    {
                        cout << "Waiting for image from topic: " << "/iitktcs/foscam/image_color" << endl;
                        ros::spinOnce();
                    }

                    cout << "***** Press 'c': start capturing camera points" << endl;
                    cout << "***** Press 'ESC': for next camera view" << endl;
                    while(ros::ok() && !src_image.empty())
                    {
                        cv::imshow("Calibrated_foscam_image", src_image);

                        char c = cv::waitKey(1);
                        if(c=='c')
                        {
                            got_cloud = true;
                            break;
                        }
                        else if(c==27)
                            break;
                        ros::spinOnce();
                    }
                }
            }
            cv::destroyWindow("Calibrated_foscam_image");
            break;
        }
        case 4:
        {
            cv::namedWindow("Select_object_bbox", cv::WINDOW_NORMAL);
            setMouseCallback("Select_object_bbox", CallBackFunc, NULL);

            visualization::PCLVisualizer viewer("cloud");
            viewer.addCoordinateSystem(0.1);
            viewer.setBackgroundColor(0, 0, 0);
            viewer.initCameraParameters();
            viewer.addPointCloud(src_cloud);

            while(ros::ok())
            {
                viewer.spinOnce();
                ros::spinOnce();

                if(!src_image.empty())
                {
                    cv::imshow("Select_object_bbox", src_image);
                    cv::waitKey(1);
                }
                if(got_object_bbox)
                {
                    got_object_bbox = false;

                    cloud_bbox->clear();
                    image_bbox.release();
                    image_bbox.create(src_image.rows,src_image.cols,CV_8UC3);
                    cloud_bbox->is_dense = cloud_bbox->is_dense;
                    for(int i=top(0); i<=bottom(0);i++)
                        for(int j=top(1); j<=bottom(1);j++)
                        {
                            cloud_bbox->points.push_back(src_cloud->at(j,i));
                            image_bbox.at<Vec3b>(i,j) = src_image.at<Vec3b>(i,j);
                        }
                    cloud_bbox->width = cloud_bbox->points.size()/abs(bottom(1)-top(1)+1);
                    cloud_bbox->height = abs(bottom(1)-top(1)+1);
                    cout << "No of points: " << cloud_bbox->points.size();
                    cout << "width: " << cloud_bbox->width << " height: " << cloud_bbox->height << endl;

                    cout << "Object BBox cloud and image is copied" << endl;
                    break;
                }
            }
            cout << "Enter object id: ";
            cin >> id_object;
            getchar();
            cv::destroyWindow("Select_object_bbox");
            viewer.close();
            break;
        }
        case 5:
        {
            iitktcs_msgs_srvs::pose estimate_pose;
            iitktcs_msgs_srvs::objects_info object_info;

            pcl::toROSMsg(*cloud_bbox, object_info.roi);

            object_info.id.data = id_object;

            estimate_pose.request.object_info.push_back(object_info);

            if(client_object_pose_estimate.call(estimate_pose))
            {
                cout << "Found pose successfully" << endl;

                tf::StampedTransform transform = getSensorToBaseFrame();

                cout << "Pose wrt camera link" << endl;
                cout << estimate_pose.response.pose.position.x << " "
                     << estimate_pose.response.pose.position.y << " "
                     << estimate_pose.response.pose.position.z << " "
                     << estimate_pose.response.pose.orientation.x << " "
                     << estimate_pose.response.pose.orientation.y << " "
                     << estimate_pose.response.pose.orientation.z << " "
                     << estimate_pose.response.pose.orientation.w << endl;

                target_object_collision_pose = estimate_pose.response.pose;

                Eigen::Affine3d affine_matrix;
                tf::poseMsgToEigen(target_object_collision_pose, affine_matrix);
                target_object_collision_pose.position = transformSensorToWorldFrame(transform, estimate_pose.response.pose.position);

                affine_matrix(0,3) = target_object_collision_pose.position.x;
                affine_matrix(1,3) = target_object_collision_pose.position.y;
                affine_matrix(2,3) = target_object_collision_pose.position.z;
                for(int i=0; i<3; i++)
                {
                    geometry_msgs::Point v;
                    v.x = affine_matrix(0,i);
                    v.y = affine_matrix(1,i);
                    v.z = affine_matrix(2,i);
                    geometry_msgs::Point tf_v = transformSensorToWorldFrame(transform, v, true);

                    affine_matrix(0,i) = tf_v.x;
                    affine_matrix(1,i) = tf_v.y;
                    affine_matrix(2,i) = tf_v.z;
                }
                tf::poseEigenToMsg(affine_matrix, target_object_collision_pose);

                cout << "Pose wrt world link" << endl;
                cout << target_object_collision_pose.position.x << " "
                     << target_object_collision_pose.position.y << " "
                     << target_object_collision_pose.position.z << " "
                     << target_object_collision_pose.orientation.x << " "
                     << target_object_collision_pose.orientation.y << " "
                     << target_object_collision_pose.orientation.z << " "
                     << target_object_collision_pose.orientation.w << endl;

                target_object_centroid = transformSensorToWorldFrame(transform, estimate_pose.response.centroid);
                target_object_normal = transformSensorToWorldFrame(transform, estimate_pose.response.normal, true);
                target_object_axis = transformSensorToWorldFrame(transform, estimate_pose.response.axis, true);
            }
            else
            {
                cout << "Failed to find pose" << endl;
            }
            break;
        }
        case 6:
        {
            iitktcs_msgs_srvs::UR5PoseGoal pose_goal;

            pose_goal.request.axis = target_object_axis;
            pose_goal.request.normal = target_object_normal;
            pose_goal.request.centroid = target_object_centroid;
            pose_goal.request.object_pose = target_object_collision_pose;
            pose_goal.request.object_id.data = id_object;

            if(client_suction_pose_goal.call(pose_goal))
            {
                cout << "Successfully added collision object " << endl;
            }
            else
                cout << "Failed to call suction pose goal" << endl;
            break;
        }
        case 7:
        {
            iitktcs_msgs_srvs::computer_vision_stowing_picking object_detection;
            string object[] = {"dumbell", "toilet_brush", "scissor", "mesh_cup"};

            int bin_id, target_id;
            cout << "Enter bin_id: ";
            cin >> bin_id;
            cout << " Enter target obj id: ";
            cin >> target_id;
            object_detection.request.bin_id.data = bin_id;
            object_detection.request.ids_available.data.push_back((double) target_id);
            object_detection.request.ids_target.data.push_back((double) target_id);
            object_detection.request.task.data = "PICK";

            if(client_object_detection.call(object_detection))
            {
                iitktcs_msgs_srvs::objects_info objects_info;
                if(object_detection.response.object_info.size() > 0)
                {
                    objects_info = object_detection.response.object_info[0];
                    cloud_object_roi = objects_info.roi;
                    cout << "Object roi found " << endl;
                    cv::Mat image;
                    sensor_msgs::ImagePtr image_msg_ptr;
                    pcl::PointCloud<pcl::PointXYZRGBA> cloudrgba;
                    pcl::toROSMsg(cloud_object_roi, *image_msg_ptr);
                    pcl::fromROSMsg(cloud_object_roi, cloudrgba);
                    bool got_image = true;
                    cv_bridge::CvImagePtr cv_image_ptr;
                    try
                    {
                        cv_image_ptr = cv_bridge::toCvCopy(image_msg_ptr,"bgr8");
                    }
                    catch(cv_bridge::Exception &e)
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        got_image = false;
                    }
                    if(got_image)
                        image=cv::Mat(cv_image_ptr->image);
                    string path = "package://iitktcs_controller";
                    char s[200];
                    sprintf(s,"/data/img_%d.jpg",count);
                    string img_path = path, pcd_path = path;
                    img_path.append(s);
                    cv::imwrite(img_path, image);

                    sprintf(s,"/data/pcd_%d.pcd",count);
                    pcd_path.append(s);
                    pcl::io::savePCDFileASCII(pcd_path, cloudrgba);
                }
                else
                    cout << "Failed to detect object" << endl;
            }
        }
        case 8:
        {
            iitktcs_msgs_srvs::GetFK getFK;
            getFK.request.current_jts.data = true;
            getFK.request.end_effector_link.data = "s_st_eef_link";
            if(client_forward_kinematics.call(getFK))
            {
                tf::poseMsgToEigen(getFK.response.pose, suction_eef);

                cout << suction_eef(0,0) << " " << suction_eef(0,1) << " " << suction_eef(0,2) << " " << suction_eef(0,3) << endl
                     << suction_eef(1,0) << " " << suction_eef(1,1) << " " << suction_eef(1,2) << " " << suction_eef(1,3) << endl
                     << suction_eef(2,0) << " " << suction_eef(2,1) << " " << suction_eef(2,2) << " " << suction_eef(2,3) << endl;
            }
            else
            {
                cout << "Failed to find forward kinematics" << endl;
            }
            break;
        }
        case 9:
        {
            iitktcs_msgs_srvs::GetFK getFK;
            getFK.request.current_jts.data = true;
            getFK.request.end_effector_link.data = "gripper_eef_link";
            if(client_forward_kinematics.call(getFK))
            {
                tf::poseMsgToEigen(getFK.response.pose, gripper_eef);

                cout << gripper_eef(0,0) << " " << gripper_eef(0,1) << " " << gripper_eef(0,2) << " " << gripper_eef(0,3)
                     << gripper_eef(1,0) << " " << gripper_eef(1,1) << " " << gripper_eef(1,2) << " " << gripper_eef(1,3)
                     << gripper_eef(2,0) << " " << gripper_eef(2,1) << " " << gripper_eef(2,2) << " " << gripper_eef(2,3) << endl;
            }
            else
            {
                cout << "Failed to find forward kinematics" << endl;
            }
            break;
        }
        case 10:
        {
            char c;
            cout << "Press s: For suction simple pose goal" << endl;
            cout << "Press g: For gripper simple pose goal" << endl;
            cin >> c;
            getchar();
            iitktcs_msgs_srvs::UR5PoseGoal pose_goal;
            if(c=='s')
            {
                pose_goal.request.axis.x = suction_eef(0,0);
                pose_goal.request.axis.y = suction_eef(1,0);
                pose_goal.request.axis.z = suction_eef(2,0);

                pose_goal.request.normal.x = suction_eef(0,1);
                pose_goal.request.normal.y = suction_eef(1,1);
                pose_goal.request.normal.z = suction_eef(2,1);

                pose_goal.request.centroid.x = suction_eef(0,3);
                pose_goal.request.centroid.y = suction_eef(1,3);
                pose_goal.request.centroid.z = suction_eef(2,3);

                pose_goal.request.end_effector_link.data = "s_eef_link";
            }
            else if(c=='g')
            {
                pose_goal.request.axis.x = gripper_eef(0,0);
                pose_goal.request.axis.y = gripper_eef(1,0);
                pose_goal.request.axis.z = gripper_eef(2,0);

                pose_goal.request.normal.x = gripper_eef(0,1);
                pose_goal.request.normal.y = gripper_eef(1,1);
                pose_goal.request.normal.z = gripper_eef(2,1);

                pose_goal.request.centroid.x = gripper_eef(0,3);
                pose_goal.request.centroid.y = gripper_eef(1,3);
                pose_goal.request.centroid.z = gripper_eef(2,3);

                pose_goal.request.end_effector_link.data = "gripper_eef_link";
            }

            if(client_simple_pose_goal.call(pose_goal))
            {
                cout << "Success " << endl;
            }
            else
            {
                cout << "Failed" << endl;
            }
            break;
        }

        }
    }


    return 0;
}
