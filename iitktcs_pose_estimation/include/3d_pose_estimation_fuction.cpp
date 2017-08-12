

EstimatePose::EstimatePose()
{
    model_fit = nh.advertiseService("/iitktcs/estimate_pose",&EstimatePose::callBack,this);
    nh.getParam("/bin_corners0",bin_corner0);
    nh.getParam("/bin_corners1",bin_corner1);
    repose = nh.advertiseService("/iitktcs/re_pose",&EstimatePose::rePosecallBack,this);
    tree = search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ> ());
    scene = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());
    input_scene = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());
    
}

EstimatePose::~EstimatePose()
{
    
    
    
}

bool EstimatePose::callBack(ensenso_testing::pose::Request &req, ensenso_testing::pose::Response &res)
{
    //    string model_names[] = {"final_dumbell","final_baby_swipes","final_speed_stick","final_irish_spring","final_toothbrush","final_balls","towel1","towel2","lufa"};
    
    const std::string model_names[]=
    {
        "Toilet_Brush","Avery_Binder","Balloons","Band_Aid_Tape","Scotch_Sponges","Black_Fashion_Gloves","Burts_Bees_Baby_Wipes","Colgate_Toothbrush_4PK","Composition_Book","Crayons"
        ,"Duct_Tape","Epsom_Salts","Expo_Eraser","Fiskars_Scissors","Flashlight","Glue_Sticks","Hand_Weight","Hanes_Socks" ,"Hinged_Ruled_Index_Cards","Ice_Cube_Tray","Irish_Spring_Soap"
        ,"Laugh_Out_Loud_Jokes","Marbles","Measuring_Spoons","Mesh_Cup","Mouse_Traps","Pie_Plates","Plastic_Wine_Glass","Poland_Spring_Water","Reynolds_Wrap","Robots_DVD"
        ,"Robots_Everywhere","Bath_Sponge","Speed_Stick","White_Facecloth","Table_Cloth","Tennis_Ball_Container","Ticonderoga_Pencils","Tissue_Box","Windex"
    };
    
    
    int geometry_type[] =
    {
        0,1,1,1,2,1,1,1,1,1,
        0,1,1,1,0,1,0,1,1,1,1,
        1,1,1,0,1,1,0,0,1,1,
        1,1,0,1,1,0,1,0,1
        
    };/////////// 1 means plane type, 0 signifies cylinder type and 2 for sphere type
    int object_type[] =
    {
        1,1,0,1,1,0,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,
        1,0,1,1,1,1,1,1,1,1,
        1,0,1,0,0,1,1,1,1
    };//// 1 signifies object is formable and 0 signifies oject is deformable type
    
    
    double object_height_offset[]=
    {
        0,0.041,0,0,0.06,0,0.056,0,0,0.028,
        0,0.068,0,0,0.025,0.022,0.057,0,0,0,0.039,
        0.015,0,0,0,0,0,0,0,0.047,0,
        0,0.053,0.061,0,0,0.077,0,0.113,0.061
        
    };
    
    double object_length_offset[]=
    {
        0,0.266,0,0,0.145,0,0.225,0,0,0,
        0,0,0.135,0,0.085,0,0.144,0,0,0,0.100,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0.144,0,0.218,0,0.113,0,0


    };

    double object_width_offset[]=
    {
        0,0.294,0,0,0.082,0,0.119,0,0,0,
        0,0,0.055,0,0.025,0,0.059,0,0,0,0.064,
        0,0,0,0,0,0,0,0,0.47,0,
        0,0,0.069,0,0.077,0,0.116,0,0
    };
    
    vector<ensenso_testing::objects_info> roi_objects;
    
    sensor_msgs::PointCloud2 cloud_ros;
    cout << "Response size: " << req.object_info.size() << endl;
    
    for(int i=0;i<req.object_info.size();i++)
    {
        roi_objects.push_back(req.object_info[i]);
        cout << "object ids: " << roi_objects[i].id.data << endl;
    }
    
    object_id = roi_objects[0].id.data -1;
    
    
    bin_id = req.bin_id.data;
    cout << "bin_id: " << bin_id << endl;
    cout << "object_id: " << object_id;
    modelName = model_names[object_id];
    cout << "Model Name: " << modelName << endl;
    cloud_ros = roi_objects[0].roi;
    fromROSMsg(cloud_ros,*scene);
    if(!(scene->size()==0))
    {
        copyPointCloud(*scene,*input_scene);
        pcl::visualization::PCLVisualizer vis("roi");
        vis.addPointCloud(scene);
        while(!vis.wasStopped())
            vis.spinOnce();

        height_offset = object_height_offset[object_id];
        length_offset = object_length_offset[object_id];
        width_offset = object_width_offset[object_id];

        int flag;
        flag = geometry_type[object_id];
        cout << "flag value" << flag << endl;

#if(TRANSFORM)

        tf::StampedTransform transform;
        transform = getSensorToBaseFrame();
        Eigen::Affine3d pt_transform;

        tf::transformTFToEigen(transform,pt_transform);
        pcl::transformPointCloud(*scene,*scene,pt_transform);

#endif

        if(object_type[object_id]==1)
            mainFuctionFormable(flag);
        else
            mainFuctionDeFormable(flag);


        geometry_msgs::Pose mat_super4pcs_pose;
        //    Eigen::Affine3d p;
        tf::poseEigenToMsg(mat_super4pcs,mat_super4pcs_pose);

        res.pose = mat_super4pcs_pose;
        res.centroid.x = final_centroid(0); res.centroid.y = final_centroid(1); res.centroid.z = final_centroid(2);
        res.axis.x = final_axis(0); res.axis.y = final_axis(1); res.axis.z = final_axis(2);
        res.normal.x = final_normal(0); res.normal.y = final_normal(1); res.normal.z = final_normal(2);
        res.axis1.x = final_axis1(0); res.axis1.y = final_axis1(1); res.axis1.z = final_axis1(2);
        res.angle.data = angle;

        roi_objects.clear();
        ss_model_obj.str("");
        ss_scene_obj.str("");
        ss_model.str("");
        ss_super4pcs.str("");
        ss_super4pcs_matrix.str("");
    }
    else
        cout << "input cloud is empty" << endl;
    
    
    return true;
    
}


bool EstimatePose::mainFuctionFormable(int flag)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    double leafsize[3] = {0.02,0.02,.02};
    downSampleCloudVoxel(scene,scene_downsampled,leafsize);
    path = ros::package::getPath("ensenso_testing");
    ss_scene_obj << path << "/data_new/" << modelName << "_scene.obj";
    createObjFile(scene_downsampled,ss_scene_obj.str());
    
    PointCloud<PointXYZ>::Ptr model(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr model_downsampled(new PointCloud<PointXYZ>());
    ss_model << path << "/artificial_pcd/" << modelName << ".pcd";
    pcl::io::loadPCDFile(ss_model.str().c_str(),*model);
    downSampleCloudVoxel(model,model_downsampled,leafsize);
    ss_model_obj << path <<"/data_new/" << modelName << ".obj";
    createObjFile(model_downsampled,ss_model_obj.str());
    
    applySuper4pcs();
    PointCloud<PointXYZ>::Ptr model_transformed(new PointCloud<PointXYZ>());
    pcl::transformPointCloud(*model,*model_transformed,mat_super4pcs);
    
    pcl::visualization::PCLVisualizer vis("check");
    vis.addPointCloud (model_transformed, ColorHandlerT (model_transformed, 0.0, 255.0, 0.0), "scene_allignment");
    vis.addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object");
    vis.spin();
    vis.close();
    
    
    newcode(model_transformed,flag);
    //    pcl::visualization::PCLVisualizer vis()
    //   / findNormalAxis(model_transformed);
    
    
    //    showPointCloud(model_transformed,scene);
    
    
    
    
    //    vector<int> index;
    //    pcl::removeNaNFromPointCloud(*scene_downsampled,*scene_downsampled,index);
    //    PointCloud<PointXYZ>::Ptr segmented_scene(new PointCloud<PointXYZ>());
    //    segmentScene(scene_downsampled,model_transformed,segmented_scene);
    
    
    //    PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>());
    //    //    computeNormal(segmented_scene,scene_normals);
    //    computeNormal(scene_downsampled,scene_normals);
    
    //    Eigen::Vector3f normal;
    //    if(flag==1)
    //        planFit(scene_downsampled,scene_normals,normal);
    //    else if (flag == 0)
    //        cylinderFit(scene_downsampled,scene_normals,normal);
    //    else
    //        cout << "Error in setting flag" <<  endl;
    
    
}

bool EstimatePose::mainFuctionDeFormable(int flag)
{
    cout << "flag value: " << flag << endl;
    vector<int> index;
    pcl::removeNaNFromPointCloud(*input_scene,*input_scene,index);
    
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(input_scene);
#if(TRANSFORM)
    
    tf::StampedTransform transform;
    transform = getSensorToBaseFrame();
    Eigen::Affine3d pt_transform;
    
    tf::transformTFToEigen(transform,pt_transform);
    pcl::transformPointCloud(*input_scene,*input_scene,pt_transform);
    
#endif
    
    
    
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f axis = eigen_vectors.col(0);
    
    Eigen::Vector4d centroid_compute;
    pcl::compute3DCentroid(*input_scene,centroid_compute);
    Eigen::Vector3f centroid;
    centroid(0) = centroid_compute(0);
    centroid(1) = centroid_compute(1);
    centroid(2) = centroid_compute(2);
    
    final_centroid = centroid;
    final_axis = axis;
    
    PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>());
    computeNormal(input_scene,scene_normals);
    
    Eigen::Vector3f normal;
    if(flag==1)
        planFit(input_scene,scene_normals,normal);
    else if (flag == 0)
        cylinderFit(input_scene,scene_normals,normal);
    else
        sphereFit(input_scene,scene_normals,normal);
    
    
    normal = normal*(-1);
    final_normal = normal;
    
}


void EstimatePose::createObjFile(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,string a)
{
    ofstream os(a.c_str());
    
    for(int i=0 ; i<cloud->size(); i++)
    {
        if(!std::isnan(cloud->points[i].x))
        {
            os << "v ";
            os << cloud->points[i].x << " ";
            os << cloud->points[i].y << " ";
            os << cloud->points[i].z << "\n";
        }
    }
    
    os.close();
    
    
}

void EstimatePose::downSampleCloudVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr,double* leafsize)
{
    //    cout << "leafsize"  << leafsize[0] << " " << leafsize[2] << " " << leafsize[2] << endl;
    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leafsize[0], leafsize[1], leafsize[2]);
    sor.filter (*cloud_ptr);
    
}



void EstimatePose::applySuper4pcs()
{
    ss_super4pcs << "time -p /home/manish/Super4PCS-master/build/Super4PCS -i " << ss_scene_obj.str().c_str() << " " << ss_model_obj.str().c_str() << " -o 0.9 -d 0.01 -t 1000 -n 100 -r super4pcs_fast.obj -m "<< path << "/data/mat_super4pcs_fast.txt";
    system(ss_super4pcs.str().c_str());
    
    ss_super4pcs_matrix << path << "/data/mat_super4pcs_fast.txt";
    ifstream input_file(ss_super4pcs_matrix.str().c_str());
    input_file.seekg(0,ios::end);
    input_file.seekg(20,ios::beg);
    
    vector<double> temp(16);
    
    for(int i=0;i<16;i++)
    {
        input_file >> temp[i];
        cout << temp[i] << endl;
        
    }
    
    int w=0;
    Eigen::Matrix4d mat;
    for(int i=0; i<4; i++)
        for(int j=0;j<4;j++)
        {
            mat(i,j) = temp[w];
            w++;
        }
    
    mat_super4pcs.matrix() = mat;
    
    
}

void EstimatePose::showPointCloud(PointCloud<PointXYZ>::Ptr &model, PointCloud<PointXYZ>::Ptr &scene)
{
    pcl::visualization::PCLVisualizer::Ptr visu ( new pcl::visualization::PCLVisualizer("Alignment_Super4PCS"));
    visu->addPointCloud (model, ColorHandlerT (model, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object_aligned");
    axisShow(model,visu);
    while(!visu->wasStopped())
        visu->spinOnce();
    //    visu->spin ();
    visu->close();
    
}

void EstimatePose::axisShow(PointCloud<PointXYZ>::Ptr &transformed_model, visualization::PCLVisualizer::Ptr &visu)
{
    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*transformed_model,cn);
    Eigen::Vector3f centroid_model_new;
    centroid_model_new(0) = cn(0); centroid_model_new(1) = cn(1); centroid_model_new(2) = cn(2);
    pcl::PCA<pcl::PointXYZ> pca5;
    pca5.setInputCloud(transformed_model);
    Eigen::Matrix3f new_model_eigen_vectors = pca5.getEigenVectors();
    Eigen::Vector3f mazor_axis_point,minor_axis_point,third_axis_point;
    Eigen::Vector3f model_major_axis,model_minor_axis,model_third_axis;
    model_major_axis = new_model_eigen_vectors.col(0);
    model_minor_axis = new_model_eigen_vectors.col(1);
    model_third_axis = new_model_eigen_vectors.col(2);
    
    Eigen::Vector3f r_major_point,r_minor_point,r_third_point,Normal;
    Eigen::Vector3f r_major  =model_major_axis;
    Eigen::Vector3f r_minor = model_minor_axis;
    Eigen::Vector3f r_third = model_third_axis;
    
    //    pcl::PointXYZ pt;
    //    pt.x = cn(0);  pt.y = cn(1);  pt.z = cn(2);
    //    pcl::flipNormalTowardsViewpoint(pt,0,0,0,r_major(0),r_major(1),r_major(2));
    //    pcl::flipNormalTowardsViewpoint(pt,0,0,0,r_minor(0),r_minor(1),r_minor(2));
    //    pcl::flipNormalTowardsViewpoint(pt,0,0,0,r_third(0),r_third(1),r_third(2));
    
    mazor_axis_point = centroid_model_new + 0.15*model_major_axis;
    minor_axis_point = centroid_model_new + 0.15 *model_minor_axis;
    third_axis_point = centroid_model_new + 0.15*model_third_axis;
    
    r_major_point = centroid_model_new + 0.15*r_major;
    r_minor_point = centroid_model_new + 0.15*r_minor;
    r_third_point = centroid_model_new + 0.15*r_third;
    
    
    pcl::PointXYZ centroid_arrow,arrow_major,arrow_minor,arrow_third;
    centroid_arrow.x = centroid_model_new(0); centroid_arrow.y = centroid_model_new(1); centroid_arrow.z = centroid_model_new(2);
    arrow_major.x = mazor_axis_point(0); arrow_major.y = mazor_axis_point(1); arrow_major.z = mazor_axis_point(2);
    arrow_minor.x = minor_axis_point(0); arrow_minor.y = minor_axis_point(1); arrow_minor.z = minor_axis_point(2);
    arrow_third.x = third_axis_point(0); arrow_third.y = third_axis_point(1); arrow_third.z = third_axis_point(2);
    
    pcl::PointXYZ r_arrow_major,r_arrow_minor,r_arrow_third;
    r_arrow_major.x = r_major_point(0); r_arrow_major.y = r_major_point(1);r_arrow_major.z = r_major_point(2);
    r_arrow_minor.x = r_minor_point(0); r_arrow_minor.y = r_minor_point(1);r_arrow_minor.z = r_minor_point(2);
    r_arrow_third.x = r_third_point(0); r_arrow_third.y = r_third_point(1);r_arrow_third.z = r_third_point(2);
    
    //    visu.addSphere<pcl::PointXYZ>(centroid_arrow,0.025,"sphere1");
    //    visu.addSphere<pcl::PointXYZ>(arrow_major,0.025,"sphere2");
    visu->addArrow<pcl::PointXYZ>(arrow_major,centroid_arrow,1,0,0,false,"arrow");
    visu->addArrow<pcl::PointXYZ>(arrow_minor,centroid_arrow,0,1,0,false,"arrow_wq");
    visu->addArrow<pcl::PointXYZ>(arrow_third,centroid_arrow,0,0,1,false,"arrow_wqqq");
    visu->addArrow<pcl::PointXYZ>(r_arrow_major,centroid_arrow,0.5,0.5,0,false,"r_major");
    visu->addArrow<pcl::PointXYZ>(r_arrow_minor,centroid_arrow,0,0.5,0.5,false,"r_minor");
    visu->addArrow<pcl::PointXYZ>(r_arrow_third,centroid_arrow,0.5,0,0.5,false,"r_third");
    //    visu->addSphere<pcl::PointXYZ>(centroid_arrow,0.025,"sphere");
    visu->addCoordinateSystem(0.5,0,0,0);
    
    
    //    double a = centroid_model_new.dot(r_major);
    //    double b = centroid_model_new.dot(r_minor);
    //    double c  = centroid_model_new.dot(r_third);
    
    double a = r_major(2);
    double b = r_minor(2);
    double c = r_third(2);
    
    cout << "value of a b c: " << fabs(a) << " " << fabs(b) << " " << fabs(c) << endl;
    
    if(fabs(b) > fabs((c)))
    {
        Normal = centroid_model_new + 0.5*r_minor;
        pcl::PointXYZ normal_point;
        normal_point.x = Normal(0); normal_point.y = Normal(1); normal_point.z = Normal(2);
        
        //        visu->addArrow<pcl::PointXYZ>(normal_point,centroid_arrow,1,1,1,false,"Normal");
        final_normal = r_minor;
    }
    else
    {
        Normal = centroid_model_new + 0.5*r_third;
        pcl::PointXYZ normal_point;
        normal_point.x = Normal(0); normal_point.y = Normal(1); normal_point.z = Normal(2);
        
        //        visu->addArrow<pcl::PointXYZ>(normal_point,centroid_arrow,1,1,1,false,"Normal");
        final_normal = r_third;
        
    }
    
    
    if(final_normal(2)>cn(2))
    {
        cout << "z part of Normal: " << final_normal(2) << endl;
        final_normal.normalize();
        cout << "Normal remains same" << endl;
    }
    else
    {
        cout << "z part of Normal: " << final_normal(2) << endl;
        final_normal = final_normal*(-1);
        final_normal.normalize();
        cout << "Normal Direction changed" << endl;
    }
    
    
    final_centroid = centroid_model_new + (height_offset/2)*final_normal;
    final_axis = model_major_axis;
    
    pcl::PointXYZ fc;
    fc.x = final_centroid(0);
    fc.y = final_centroid(1);
    fc.z = final_centroid(2);
    
    Normal = centroid_model_new + 0.5*final_normal;
    pcl::PointXYZ normal_point;
    normal_point.x = Normal(0); normal_point.y = Normal(1); normal_point.z = Normal(2);
    
    visu->addArrow<pcl::PointXYZ>(normal_point,centroid_arrow,1,1,1,false,"Normal");
    
    visu->addSphere<pcl::PointXYZ>(fc,0.025,"sphere");
    
    
    //    Eigen::Vector3f bin_a;
    //    Eigen::Vector3f bin_b;
    //    Eigen::Vector3f bin_normal;
    
    //    if(bin_id==0)
    //    {
    //        bin_a(0) = bin_corner0[3] - bin_corner0[0];
    //        bin_a(1) = bin_corner0[4] - bin_corner0[1];
    //        bin_a(2) = bin_corner0[5] - bin_corner0[2];
    
    //        bin_b(0) = bin_corner0[0] - bin_corner0[6];
    //        bin_b(1) = bin_corner0[1] - bin_corner0[7];
    //        bin_b(2) = bin_corner0[2] - bin_corner0[8];
    
    //        bin_normal = bin_a.cross(bin_b);
    
    //    }
    
    //    if(bin_id==1)
    //    {
    //        bin_a(0) = bin_corner1[3] - bin_corner1[0];
    //        bin_a(1) = bin_corner1[4] - bin_corner1[1];
    //        bin_a(2) = bin_corner1[5] - bin_corner1[2];
    
    //        bin_b(0) = bin_corner1[0] - bin_corner1[6];
    //        bin_b(1) = bin_corner1[1] - bin_corner1[7];
    //        bin_b(2) = bin_corner1[2] - bin_corner1[8];
    
    //        bin_normal = bin_a.cross(bin_b);
    
    //   }
    
    //   final_axis = bin_b;
    
    //   cout << "bin_b : " << bin_b(0) << " "<< bin_b(1) << " "<< bin_b(2) <<endl;
    
    
    
    
}


void EstimatePose::segmentScene(PointCloud<PointXYZ>::Ptr &nan_free_scene,PointCloud<PointXYZ>::Ptr &model,PointCloud<PointXYZ>::Ptr &segmented_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ> search;
    double dist = 0.01;
    search.setInputCloud(nan_free_scene);
    
    segmented_cloud->header = nan_free_scene->header;
    
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    std::vector<int> indices;
    
    int cloud_size  = model->size();
    for(int i=0;i<cloud_size;i++)
    {
        pcl::PointXYZ pt;
        pt = model->at(i);
        search.radiusSearch(pt,dist,nn_indices,nn_dists);
        for(int j=0;j<nn_indices.size();j++)
            indices.push_back(nn_indices[j]);
    }
    
    // add nearby indices to the segmented cloud
    for (int i=0;i<indices.size();i++)
    {
        segmented_cloud->points.push_back(nan_free_scene->points[indices[i]]);
    }
    
    pcl::visualization::PCLVisualizer viewer("segmented cloud");
    viewer.addPointCloud(segmented_cloud);
    viewer.spin();
    viewer.close();
    
    
}


void EstimatePose::computeNormal(PointCloud<PointXYZ>::Ptr &scene, PointCloud<Normal>::Ptr &scene_normal)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (scene);
    ne.setKSearch (50);
    ne.compute (*scene_normal);
    
}


void EstimatePose::planFit(PointCloud<PointXYZ>::Ptr &scene, PointCloud<pcl::Normal>::Ptr &scene_normals, Eigen::Vector3f &Normal)
{
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (scene);
    seg.setInputNormals (scene_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    
    Normal(0) = coefficients_plane->values[0];
    Normal(1) = coefficients_plane->values[1];
    Normal(2) = coefficients_plane->values[2];
    
    
    
    projection(scene,coefficients_plane);
    
}



void EstimatePose::cylinderFit(PointCloud<PointXYZ>::Ptr &scene,PointCloud<pcl::Normal>::Ptr &scene_normals,Eigen::Vector3f &Normal)
{
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (scene);
    seg.setInputNormals (scene_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    cylinderNormal(scene,coefficients_cylinder,Normal);
    
    string id = "Normal12";
    pcl::visualization::PCLVisualizer::Ptr  v (new pcl::visualization::PCLVisualizer("segmented scene"));
    v->addPointCloud(input_scene);
    coefficients_cylinder->values[3]/=5;
    coefficients_cylinder->values[4]/=5;
    coefficients_cylinder->values[5]/=5;
    v->addCylinder(*coefficients_cylinder,"cylinder");
    arrowvisulizer(scene,Normal,v,id);
    v->spin();
    v->close();
    
}


void EstimatePose::sphereFit(PointCloud<PointXYZ>::Ptr &scene,PointCloud<pcl::Normal>::Ptr &scene_normals,Eigen::Vector3f &Normal)
{
    pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (scene);
    seg.setInputNormals (scene_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_sphere, *coefficients_sphere);
    
    std::cerr << "Cylinder coefficients: " << *coefficients_sphere << std::endl;
    
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*scene,centroid);
    
    pcl::PointXYZ pt;
    pt.x = centroid(0); pt.y = centroid(1); pt.z = centroid(2);
    
    Normal(0) = centroid(0) - coefficients_sphere->values[0];
    Normal(1) = centroid(1) - coefficients_sphere->values[1];
    Normal(2) = centroid(2) - coefficients_sphere->values[2];
    pcl::flipNormalTowardsViewpoint(pt,0,0,0,Normal(0),Normal(1),Normal(2));
    
    Normal.normalize();
    
    string id = "Normal12";
    string id1 = "axis";
    pcl::visualization::PCLVisualizer::Ptr  v (new pcl::visualization::PCLVisualizer("spherefit"));
    v->addPointCloud(input_scene);
    v->addSphere(*coefficients_sphere,"sphere");
    arrowvisulizer(scene,Normal,v,id);
    arrowvisulizer(scene,final_axis,v,id1);
    v->spin();
    v->close();
    
}







void EstimatePose::cylinderNormal(PointCloud<PointXYZ>::Ptr &scene, ModelCoefficients::Ptr &coe, Eigen::Vector3f &Normal)
{
    
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*scene,centroid);
    
    float x = centroid(0);
    float y = centroid(1);
    float z = centroid(2);
    
    float xo = coe->values[0];
    float yo = coe->values[1];
    float zo = coe->values[2];
    
    float a = coe->values[3];
    float b = coe->values[4];
    float c = coe->values[5];
    
    float t = (a*(x-xo)+b*(y-yo)+c*(z-zo))/(a*a+b*b+c*c);
    
    Normal(0) = (x-xo-a*t);
    Normal(1) = (y-yo-b*t);
    Normal(2) = (z-zo-c*t);
    Normal.normalize();
    
    pcl::PointXYZ pt;
    pt.x = x; pt.y = y; pt.z =z;
    pcl::flipNormalTowardsViewpoint(pt,0,0,0,Normal(0),Normal(1),Normal(2));
    
}




void EstimatePose::projection(PointCloud<PointXYZ>::Ptr &object1,ModelCoefficients::Ptr &coe)
{
    float a = coe->values[0];
    float b = coe->values[1];
    float c = coe->values[2];
    float d = coe->values[3];
    
    Eigen::Vector3f Normal;
    Normal(0) = a;
    Normal(1) = b;
    Normal(2) = c;
    
    std::cout << "values " <<  a  <<  " "  <<  b << " " << c << " " << d << endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr projetced_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    //    cout << "object size " << object1->size() <<endl;
    
    int size  = object1->size();
    
    for(int i =0;i< size;i++)
    {
        pcl::PointXYZ pt = object1->points[i];
        float xo = pt.x;
        float yo = pt.y;
        float zo = pt.z;
        
        //        std::cout << "values " <<  xo <<  " "  <<  yo << " " << zo <<  endl;
        
        float t;
        
        t = -(d + a*xo + b*yo + c*zo)/(a*a + b*b +c*c);
        
        //        cout << "t value" << t << endl;
        
        pcl::PointXYZ projected_pt;
        projected_pt.x = a*t + xo;
        projected_pt.y = b*t + yo;
        projected_pt.z = c*t + zo;
        
        
        projetced_cloud->points.push_back(projected_pt);
        
    }
    
    
    
    
    
    
    string id = "normal";
    string id1 = "axis";
    pcl::visualization::PCLVisualizer::Ptr visu ( new pcl::visualization::PCLVisualizer("Alignment_Super4PCS"));
    visu->addPointCloud (projetced_cloud, ColorHandlerT (projetced_cloud, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (object1, ColorHandlerT (object1, 0.0, 0.0, 255.0), "object_aligned");
    visu->addPointCloud(input_scene,ColorHandlerT(input_scene,255.0,0.0,0.0),"scene1");
    visu->addCoordinateSystem(0.05,0,0,0);
    arrowvisulizer(object1,Normal,visu,id);
    arrowvisulizer(object1,final_axis,visu,id1);
    visu->spin ();
    visu->close();
    
    
}



void EstimatePose::arrowvisulizer(PointCloud<PointXYZ>::Ptr &transformed_model,Eigen::Vector3f &model_major_axis,visualization::PCLVisualizer::Ptr &visu,string a)
{
    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*transformed_model,cn);
    Eigen::Vector3f centroid_model_new;
    centroid_model_new(0) = cn(0); centroid_model_new(1) = cn(1); centroid_model_new(2) = cn(2);
    
    pcl::PointXYZ pt;
    pt.x = cn(0); pt.y = cn(1); pt.z = cn(2);
    pcl::flipNormalTowardsViewpoint(pt,0,0,0,model_major_axis(0),model_major_axis(1),model_major_axis(2));
    
    Eigen::Vector3f mazor_axis_point;
    
    mazor_axis_point = centroid_model_new + 0.20*model_major_axis;
    
    pcl::PointXYZ centroid_arrow,arrow_major,arrow_minor,arrow_third;
    centroid_arrow.x = centroid_model_new(0); centroid_arrow.y = centroid_model_new(1); centroid_arrow.z = centroid_model_new(2);
    arrow_major.x = mazor_axis_point(0); arrow_major.y = mazor_axis_point(1); arrow_major.z = mazor_axis_point(2);
    
    //    visu.addSphere<pcl::PointXYZ>(centroid_arrow,0.025,"sphere1");
    //    visu.addSphere<pcl::PointXYZ>(arrow_major,0.025,"sphere2");
    visu->addArrow<pcl::PointXYZ>(arrow_major,centroid_arrow,1,0,0,false,a);
    //    visu->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,a);
    
    
}

bool EstimatePose::rePosecallBack(ensenso_testing::repose::Request &req,ensenso_testing::repose::Response &res)
{
    Eigen::Vector3f bin_normal;
    Eigen::Vector3f bin_a;
    Eigen::Vector3f bin_b;
    
    if(req.bin_id.data==0)
    {
        bin_a(0) = bin_corner0[3] - bin_corner0[0];
        bin_a(1) = bin_corner0[4] - bin_corner0[1];
        bin_a(2) = bin_corner0[5] - bin_corner0[2];
        
        bin_b(0) = bin_corner0[0] - bin_corner0[6];
        bin_b(1) = bin_corner0[1] - bin_corner0[7];
        bin_b(2) = bin_corner0[2] - bin_corner0[8];
        
        bin_normal = bin_a.cross(bin_b);
        
    }
    
    if(req.bin_id.data==1)
    {
        bin_a(0) = bin_corner1[3] - bin_corner1[0];
        bin_a(1) = bin_corner1[4] - bin_corner1[1];
        bin_a(2) = bin_corner1[5] - bin_corner1[2];
        
        bin_b(0) = bin_corner1[0] - bin_corner1[6];
        bin_b(1) = bin_corner1[1] - bin_corner1[7];
        bin_b(2) = bin_corner1[2] - bin_corner1[8];
        
        bin_normal = bin_a.cross(bin_b);
        
    }
    
    res.re_axis.x = bin_b(0);
    res.re_axis.y = bin_b(1);
    res.re_axis.z = bin_b(2);
    
    res.re_normal.x  = bin_normal(0);
    res.re_normal.y  = bin_normal(1);
    res.re_normal.z  = bin_normal(2);
    
    res.re_centroid = req.centroid;
    res.re_pose = req.pose;
    
    return true;
    
}


#if(TRANSFORM)
tf::StampedTransform  EstimatePose::getSensorToBaseFrame()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;
    
    transform_listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);
    
    return transform;
}

// Function to transform a point or vector from sensor frame to Robot world frame
// is_vector_transform = true. Then the point is assumed to be vector
// by default is_vector_transform = false. Computes transform for the point
//tf::Vector3 EstimatePose::transformSensorToWorldFrame(tf::StampedTransform &transform,
//                                                      tf::Vector3 &point,
//                                                      bool is_vector_transform=false)
//{
//    tf::Vector3 tf_pt = transform*point;

//    tf::Vector3 tf_point = tf_pt;
//    if(is_vector_transform)
//    {
//        tf::Vector3 tr_origin = transform.getOrigin();
//        tf_point -= tr_origin;

//        return tf_point;
//    }
//    else
//    {
//        return tf_point;
//    }
//}
#endif

void EstimatePose::findNormalAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr &model)
{
    
    pcl::visualization::PCLVisualizer::Ptr visu(new pcl::visualization::PCLVisualizer("xsfNormalAxis"));
    visu->addPointCloud (model, ColorHandlerT (model, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object_aligned");
    visu->addCoordinateSystem(0.5,0,0,0);
    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*model,cn);
    Eigen::Vector3f centroid_model_new;
    centroid_model_new(0) = cn(0); centroid_model_new(1) = cn(1); centroid_model_new(2) = cn(2);
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(model);
    Eigen::Matrix3f new_model_eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f model_major_axis,model_minor_axis,model_third_axis;
    model_major_axis = new_model_eigen_vectors.col(0);
    model_minor_axis = new_model_eigen_vectors.col(1);
    model_third_axis = new_model_eigen_vectors.col(2);
    
    cout << "bin corners: " << bin_corner0[1];
    
    Eigen::Vector3f binA,binB,binC,binD,bin_left,bin_right,bin_top,bin_bottom,left_plane,right_plane,top_plane,bottom_plane;
    if(bin_id == 0)
    {
        cout << "IN BIN O" << endl;
        binA(0) = bin_corner0[0]; binA(1) = bin_corner0[1]; binA(2) = bin_corner0[2];
        binB(0) = bin_corner0[3]; binB[1] = bin_corner0[4]; binB(2) = bin_corner0[5];
        binC(0) = bin_corner0[6]; binC[1] = bin_corner0[7]; binC(2) = bin_corner0[8];
        binD(0) = bin_corner0[9]; binD[1] = bin_corner0[10]; binD(2) = bin_corner0[11];
    }
    if(bin_id == 1)
    {
        cout << "In BIN 1" << endl;
        binA(0) = bin_corner1[0]; binA(1) = bin_corner1[1]; binA(2) = bin_corner1[2];
        binB(0) = bin_corner1[3]; binB[1] = bin_corner1[4]; binB(2) = bin_corner1[5];
        binC(0) = bin_corner1[6]; binC[1] = bin_corner1[7]; binC(2) = bin_corner1[8];
        binD(0) = bin_corner1[9]; binD[1] = bin_corner1[10]; binD(2) = bin_corner1[11];
    }
    bin_left = (binA + binC)/2;
    bin_right = (binB + binD)/2;
    bin_top = (binA + binB)/2;
    bin_bottom = (binC + binD)/2;
    
    vector<Eigen::Vector3f> sphere_points;
    sphere_points.push_back(bin_left);
    sphere_points.push_back(bin_right);
    sphere_points.push_back(bin_top);
    sphere_points.push_back(bin_bottom);
    sphere_points.push_back(centroid_model_new);
    addsphere(visu,sphere_points,"sphere");
    
    
    left_plane = binA - binC;
    right_plane = binB -binD;
    top_plane = binB - binA;
    bottom_plane = binD- binC;
    
    cout << "going for the finding distance" << endl;
    
    double left_dist,right_dist,top_dist,bottom_dist;
    //        left_dist = projection_distance(top_plane,bin_left,centroid_model_new);
    //        right_dist = projection_distance(top_plane,bin_right,centroid_model_new);
    //        top_dist = projection_distance(left_plane,bin_top,centroid_model_new);
    //        bottom_dist = projection_distance(left_plane,bin_bottom,centroid_model_new);
    
    left_dist = bin_left(1) - centroid_model_new(1);
    right_dist = centroid_model_new(1) - bin_right(1);
    top_dist = bin_top(0) - centroid_model_new(0);
    bottom_dist = bin_bottom(0) - centroid_model_new(0);
    
    cout << "left distance: "  << left_dist << endl;
    cout << "right distance: "  << right_dist << endl;
    cout << "top distance: "  << top_dist << endl;
    cout << "bottom distance: "  << bottom_dist << endl;
    
    vector<Eigen::Vector3f> axies;
    if(fabs(left_dist)<0.03)
    {
        cout << "Object lies within left wall region" << endl;
        Eigen::Vector3f reference_normal(0,1,0);
        Eigen::Vector3f reference_axis(0,0,-1);
        axies = calculations(model_major_axis,model_minor_axis,model_third_axis,
                             reference_normal,reference_axis);
        final_normal = axies[0];
        final_axis = axies[1];
    }
    if(fabs(fabs(right_dist)<0.03))
    {
        cout << "Object lies within right wall region" << endl;
        Eigen::Vector3f reference_normal(0,-1,0);
        Eigen::Vector3f reference_axis(0,0,-1);
        axies = calculations(model_major_axis,model_minor_axis,model_third_axis,
                             reference_normal,reference_axis);
        final_normal = axies[0];
        final_axis = axies[1];
        
    }
    if(fabs(top_dist)<0.03)
    {
        cout << "Object lies within top wall region" << endl;
        Eigen::Vector3f reference_normal(1,0,0);
        Eigen::Vector3f reference_axis(0,0,-1);
        axies = calculations(model_major_axis,model_minor_axis,model_third_axis,
                             reference_normal,reference_axis);
        final_normal = axies[0];
        final_axis = axies[1];
        
    }
    
    if(fabs(bottom_dist)<0.03)
    {
        cout << "Object lies within bottom wall region" << endl;
        Eigen::Vector3f reference_normal(-1,0,0);
        Eigen::Vector3f reference_axis(0,0,-1);
        axies = calculations(model_major_axis,model_minor_axis,model_third_axis,
                             reference_normal,reference_axis);
        final_normal = axies[0];
        final_axis = axies[1];
        
    }
    if(fabs(right_dist)>0.03 && fabs(left_dist)>0.03 && fabs(top_dist)>0.03 && fabs(bottom_dist)>0.03)
    {
        cout << "Object lies within center region" << endl;
        Eigen::Vector3f reference_normal(0,0,-1);
        Eigen::Vector3f reference_axis(1,0,0);
        axies = calculations(model_major_axis,model_minor_axis,model_third_axis,
                             reference_normal,reference_axis);
        final_normal = axies[0];
        final_axis = axies[1];
        
    }
    
    double color[3] = {1,0,0};
    double color1[3] = {0,1,0};
    addaxis(visu,final_normal,centroid_model_new,color,"axis");
    addaxis(visu,final_axis,centroid_model_new,color1,"normal");
    visu->spin();
    visu->close();
    
}



double EstimatePose::projection_distance(Eigen::Vector3f &Normal, Eigen::Vector3f &point_plane, Eigen::Vector3f &given_point)
{
    Eigen::Vector3f temp;
    
    temp(0) = given_point(0) - point_plane(0);
    temp(1) = given_point(1) - point_plane(1);
    temp(2) = given_point(2) - point_plane(2);
    double dot_product;
    
    dot_product = temp.dot(Normal);
    
    double dist;
    dist = dot_product/Normal.norm();
    return dist;
    
}

vector<Eigen::Vector3f> EstimatePose::calculations(Eigen::Vector3f &major_axis,Eigen::Vector3f &minor_axis,Eigen::Vector3f &third_axis,Eigen::Vector3f &reference_normal,Eigen::Vector3f &reference_axis)
{
    vector<Eigen::Vector3f> axises;
    double a,b,c,d,e,f;
    a = (major_axis.dot(reference_normal));
    b = (minor_axis.dot(reference_normal));
    c = (third_axis.dot(reference_normal));
    
    d = (major_axis.dot(reference_axis));
    e = (minor_axis.dot(reference_axis));
    f = (third_axis.dot(reference_axis));
    
    
    //////////////////////Normal as mjaor axis ///////////////////////////
    
    if(fabs(a)>fabs(b) && fabs(a)>fabs(c))
    {
        if(a < 0)
            axises.push_back(major_axis);/// Normal axis choosen
        else
        {
            major_axis = major_axis*(-1);
            axises.push_back(major_axis);
            
        }
        
        
        if(fabs(e)>fabs(f))
        {
            if(e>0)
                axises.push_back(minor_axis);
            else
                axises.push_back(minor_axis*=-1);
        }
        else
        {
            if(f>0)
                axises.push_back(third_axis);
            else
                axises.push_back(third_axis*=-1);
        }
    }
    
    
    
    
    //////////////////////Normal as minor axis ///////////////////////////
    if(fabs(b)>fabs(a) && fabs(b)>fabs(c))
    {
        if(b < 0)
            axises.push_back(minor_axis);/// Normal axis choosen
        else
        {
            minor_axis = minor_axis*(-1);
            axises.push_back(minor_axis);
            
        }
        
        
        if(fabs(d)>fabs(f))
        {
            if(d>0)
                axises.push_back(major_axis);
            else
                axises.push_back(third_axis*=-1);
        }
        else
        {
            if(f>0)
                axises.push_back(third_axis);
            else
                axises.push_back(third_axis*=-1);
        }
    }
    
    //////////////////////Normal as minor axis ///////////////////////////
    
    if(fabs(c)>fabs(a) && fabs(c)>fabs(a))
    {
        if(c < 0)
            axises.push_back(third_axis);/// Normal axis choosen
        else
        {
            third_axis = third_axis*(-1);
            axises.push_back(third_axis);
            
        }
        
        
        if(fabs(d)>fabs(e))
        {
            if(d>0)
                axises.push_back(major_axis);
            else
                axises.push_back(third_axis*=-1);
        }
        else
        {
            if(e>0)
                axises.push_back(minor_axis);
            else
                axises.push_back(minor_axis*=-1);
        }
    }
    
    return axises;
    
}


void EstimatePose::addsphere(pcl::visualization::PCLVisualizer::Ptr &visu,vector<Eigen::Vector3f> &points, string id)
{
    for(int i=0;i<points.size();i++)
    {
        stringstream ss ;
        ss << id << i;
        pcl::PointXYZ pt;
        pt.x = points[i](0); pt.y = points[i](1); pt.z = points[i](2);
        visu->addSphere(pt,0.015,ss.str().c_str());
    }
}

void EstimatePose::addaxis(pcl::visualization::PCLVisualizer::Ptr &visu,Eigen::Vector3f &axis,Eigen::Vector3f &centroid,double *color,string id)
{
    Eigen::Vector3f axis_p;
    axis_p = centroid + 0.15*axis;
    
    pcl::PointXYZ pt_centroid,axis_point;
    pt_centroid.x = centroid(0); pt_centroid.y = centroid(1);pt_centroid.z = centroid(2);
    axis_point.x = axis_p(0); axis_point.y = axis_p(1); axis_point.z = axis_p(2);
    
    visu->addArrow<pcl::PointXYZ>(axis_point,pt_centroid,color[0],color[1],color[2],false,id);
}




void EstimatePose::newcode(pcl::PointCloud<pcl::PointXYZ>::Ptr &model, int flag)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (scene);
    pcl::PointXYZ searchPoint;
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    pcl::visualization::PCLVisualizer::Ptr visu(new pcl::visualization::PCLVisualizer("NormalAxis"));
    visu->addPointCloud (model, ColorHandlerT (model, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object_aligned");
    visu->addCoordinateSystem(0.5,0,0,0);


    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*model,cn);
    Eigen::Vector3f centroid,centroid_temp;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);



    const std::string model_names[]=
    {
        "Toilet_Brush","Avery_Binder","Balloons","Band_Aid_Tape","Scotch_Sponges","Black_Fashion_Gloves","Burts_Bees_Baby_Wipes","Colgate_Toothbrush_4PK","Composition_Book","Crayons"
        ,"Duct_Tape","Epsom_Salts","Expo_Eraser","Fiskars_Scissors","Flashlight","Glue_Sticks","Hand_Weight","Hanes_Socks" ,"Hinged_Ruled_Index_Cards","Ice_Cube_Tray","Irish_Spring_Soap"
        ,"Laugh_Out_Loud_Jokes","Marbles","Measuring_Spoons","Mesh_Cup","Mouse_Traps","Pie_Plates","Plastic_Wine_Glass","Poland_Spring_Water","Reynolds_Wrap","Robots_DVD"
        ,"Robots_Everywhere","Bath_Sponge","Speed_Stick","White_Facecloth","Table_Cloth","Tennis_Ball_Container","Ticonderoga_Pencils","Tissue_Box","Windex"
    };
    
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(model);


    
    Eigen::Matrix3f eigen_vects = pca.getEigenVectors();
    Eigen::Vector3f major_axis = eigen_vects.col(0);
    Eigen::Vector3f minor_axis = eigen_vects.col(1);
    Eigen::Vector3f third_axis = eigen_vects.col(2);

    
    stringstream ss_param;
    ss_param << "/" << model_names[object_id];
    
    vector<int> data_normal;
    nh.getParam(ss_param.str().c_str(), data_normal);
    
    cout << "data Normal: " << data_normal[0] << " " << data_normal[1] << " " << data_normal[2] << endl;
    Eigen::Vector3f reference_axis(0,0,1);
    Eigen::Vector3f Normal,Axis1,Axis2;

    double dot_major = major_axis.dot(reference_axis);
    double dot_minor = minor_axis.dot(reference_axis);
    double dot_third = third_axis.dot(reference_axis);

    /////////////*****Cuboidal Object*****///////////////////

    if(flag==1)
    {
        if(data_normal[0] == 0 && data_normal[1] == 0 && data_normal[2] == 1)
        {
            cout << "In one axis case" << endl;
            if(dot_third > 0)
                Normal = third_axis;
            else
                Normal = third_axis*(-1);

            centroid_temp = centroid + (height_offset/2)*Normal;

            Axis1 = major_axis;
            Axis2 = minor_axis;

        }




        if(data_normal[0] == 1 && data_normal[1] == 1 && data_normal[2] == 1)
        {
            cout << "In Three axis case" << endl;
            if(fabs(dot_major) > fabs(dot_minor) && fabs(dot_major) > fabs(dot_third))
            {
                cout << "Major Axis is choosen as a Normal" << endl;
                if(dot_major > 0)
                    Normal = major_axis;
                else
                    Normal = major_axis*(-1);

                centroid_temp = centroid + (length_offset/2)*Normal;

                Axis1 = third_axis;
                Axis2 = minor_axis;


            }

            if(fabs(dot_minor) > fabs(dot_major) && fabs(dot_minor) > fabs(dot_third))
            {
                cout << "Minor Axis is choosen as a Normal" << endl;
                if(dot_minor > 0)
                    Normal = minor_axis;
                else
                    Normal = minor_axis*(-1);

                centroid_temp = centroid + (width_offset/2)*Normal;

                Axis1 = major_axis;
                Axis2 = third_axis;


            }

            if(fabs(dot_third) > fabs(dot_minor) && fabs(dot_third) > fabs(dot_major))
            {
                cout << "Third Axis is choosen as a Normal" << endl;
                if(dot_third > 0)
                    Normal = third_axis;
                else
                    Normal = third_axis*(-1);

                centroid_temp = centroid + (height_offset/2)*Normal;


                Axis1 = major_axis;
                Axis2 = minor_axis;

            }

        }



        if(data_normal[0] == 0 && data_normal[1] == 1 && data_normal[2] == 1)
        {
            cout << "In Two axis case" << endl;
            if(fabs(dot_minor) > fabs(dot_major) && fabs(dot_minor) > fabs(dot_third))
            {
                if(dot_minor > 0)
                    Normal = minor_axis;
                else
                    Normal = minor_axis*(-1);

                centroid_temp = centroid + (width_offset/2)*Normal;


                Axis1 = major_axis;
                Axis2 = third_axis;

            }

            if(fabs(dot_third) > fabs(dot_minor) && fabs(dot_third) > fabs(dot_major))
            {
                if(dot_third > 0)
                    Normal = major_axis;
                else
                    Normal = third_axis*(-1);


                centroid_temp = centroid + (height_offset/2)*Normal;

                Axis1 = major_axis;
                Axis2 = minor_axis;

            }

        }

        searchPoint.x = centroid_temp(0);
        searchPoint.y = centroid_temp(1);
        searchPoint.z = centroid_temp(2);

        cout << "centoid befor KNN: " << centroid_temp(0) << " " << centroid_temp(1) << " " << centroid_temp(2) << endl;

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            cout << "find out the nearest neibhours : " << endl;
                      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                        std::cout << "    "  <<   scene->points[ pointIdxNKNSearch[i] ].x
                                  << " " << scene->points[ pointIdxNKNSearch[i] ].y
                                  << " " << scene->points[ pointIdxNKNSearch[i] ].z
                                  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }

        Eigen::Vector3f cent(0,0,0);

        for (int i = 0; i < pointIdxNKNSearch.size (); i++)
        {
            cent(0) += scene->points[pointIdxNKNSearch[i]].x;
            cent(1) += scene->points[pointIdxNKNSearch[i]].y;
            cent(2) += scene->points[pointIdxNKNSearch[i]].z;
        }

//        cout << "cent(0): " << cent(0) << endl;
        centroid_temp(0) = cent(0)/K;
        centroid_temp(1) = cent(1)/K;
        centroid_temp(2) = cent(2)/K;
    }


    /////////////*****Cylinder Object*****///////////////////


    else if(flag == 0)
    {
        cout << "Calculation for cylinder: " << endl;
        searchPoint.x = centroid(0);
        searchPoint.y = centroid(1);
        searchPoint.z = centroid(2);

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            cout << "find out the nearest neibhours : " << endl;
            //          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            //            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
            //                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
            //                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
            //                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }

        for (int i = 0; i < pointIdxNKNSearch.size (); i++)
        {
            centroid_temp(0) += scene->points[pointIdxNKNSearch[i]].x;
            centroid_temp(1) += scene->points[pointIdxNKNSearch[i]].y;
            centroid_temp(2) += scene->points[pointIdxNKNSearch[i]].z;

        }

        centroid_temp = centroid_temp/K;

        pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
        computeNormal(scene,scene_normals);
        pcl::Normal no;
        tf::Vector3 avg_normal_temp;
        for (int i = 0; i < pointIdxNKNSearch.size (); i++)
        {
            no = scene_normals->points[i];
            tf::Vector3 temp_normal(no.normal[0], no.normal[1], no.normal[2]);

            avg_normal_temp += temp_normal;
        }
        avg_normal_temp /= K;
        Eigen::Vector3f avg_normal;
        avg_normal(0) = avg_normal_temp.getX(); avg_normal(1) = avg_normal_temp.getY(); avg_normal(2) = avg_normal_temp.getZ();
        double dot_m = major_axis.dot(reference_axis);
        double dot_n = avg_normal.dot(reference_axis);

        if(abs(dot_m) > abs(dot_n))
        {
            if(dot_m > 0)
                Normal = major_axis;

            else
                Normal = major_axis*(-1);

            Axis1 = avg_normal;
            centroid_temp = centroid + (length_offset/2)*Normal;
        }
        else
        {
            Normal = avg_normal;
            Axis1 = major_axis;
            centroid_temp = centroid + (height_offset/2)*Normal;
        }

        Axis2 = minor_axis;

    }

    cout << "centroid points after KNN: " << centroid_temp(0) << " " << centroid_temp(1) << " " << centroid_temp(2) << endl;
    vector<Eigen::Vector3f> sphere_points;
    sphere_points.push_back(centroid_temp);
    addsphere(visu,sphere_points,"sphere22");

    double color1[3] = {1,0,0};
    double color2[3] = {0,1,0};
    double color3[3] = {0,0,1};
    addaxis(visu,Normal,centroid,color1,"Normal");
    addaxis(visu,Axis1,centroid,color2,"Axis1");
    addaxis(visu,Axis2,centroid,color3,"Axis2");


    int case_input;
    case_input = getRegion(centroid,visu);
    cout << "Case Input: " << case_input << endl;


    switch(case_input)
    {
    case 0:
    {
        cout << "In case 0" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check < 0)
        {
            Normal = top_plane.cross(left_plane);
            Axis1 =   top_plane*(-1);
            Axis2 = left_plane;
        }
        break;

    }
    case 1:
    {
        cout << "In case 1" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check > 0)
        {
            Normal = top_plane.cross(left_plane);
            Axis1 =   top_plane;
            Axis2 = left_plane;
        }
        break;
    }
    case 2:
    {
        cout << "In case 2" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check > 0)
        {
            Normal = top_plane.cross(left_plane);
            Axis1 = left_plane;
            Axis2 = top_plane;
        }
        break;

    }
    case 3:
    {
        cout << "In case 3" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check < 0)
        {
            Normal = top_plane.cross(left_plane);
            Axis1 =  left_plane;
            Axis2 = top_plane;
        }
        break;
    }
    }

    final_axis = Axis1;
    final_axis1 = Axis2;
    final_normal = Normal;
    Normal.normalize();
    Axis1.normalize();
    Axis2.normalize();

    double dot_normal = Normal.dot(reference_axis);

    angle = acos(dot_normal)*180/PI;
    cout << "angle with z axis: " << angle  << endl;

    final_centroid = centroid_temp;
    visu->spin();
    visu->close();

}


int EstimatePose::getRegion(Eigen::Vector3f &centroid,pcl::visualization::PCLVisualizer::Ptr &visu)
{
    Eigen::Vector3f binA,binB,binC,binD,bin_left,bin_right,bin_top,bin_bottom;
    if(bin_id == 0)
    {
        cout << "IN BIN O" << endl;
        binA(0) = bin_corner0[0]; binA(1) = bin_corner0[1]; binA(2) = bin_corner0[2];
        binB(0) = bin_corner0[3]; binB[1] = bin_corner0[4]; binB(2) = bin_corner0[5];
        binC(0) = bin_corner0[6]; binC[1] = bin_corner0[7]; binC(2) = bin_corner0[8];
        binD(0) = bin_corner0[9]; binD[1] = bin_corner0[10]; binD(2) = bin_corner0[11];
    }

    if(bin_id == 1)
    {
        cout << "In BIN 1" << endl;
        binA(0) = bin_corner1[0]; binA(1) = bin_corner1[1]; binA(2) = bin_corner1[2];
        binB(0) = bin_corner1[3]; binB[1] = bin_corner1[4]; binB(2) = bin_corner1[5];
        binC(0) = bin_corner1[6]; binC[1] = bin_corner1[7]; binC(2) = bin_corner1[8];
        binD(0) = bin_corner1[9]; binD[1] = bin_corner1[10]; binD(2) = bin_corner1[11];
    }

    bin_left = (binA + binC)/2;
    bin_right = (binB + binD)/2;
    bin_top = (binA + binB)/2;
    bin_bottom = (binC + binD)/2;

    vector<Eigen::Vector3f> sphere_points;
    sphere_points.push_back(bin_left);
    sphere_points.push_back(bin_right);
    sphere_points.push_back(bin_top);
    sphere_points.push_back(bin_bottom);

    addsphere(visu,sphere_points,"sphere1");

    left_plane = binA - binC;
    left_plane.normalize();
    right_plane = binB -binD;
    right_plane.normalize();
    top_plane = binB - binA;
    top_plane.normalize();
    bottom_plane = binD- binC;
    bottom_plane.normalize();

    cout << "going for the finding distance" << endl;

    double left_dist,right_dist,top_dist,bottom_dist;
    //        left_dist = projection_distance(top_plane,bin_left,centroid_model_new);
    //        right_dist = projection_distance(top_plane,bin_right,centroid_model_new);
    //        top_dist = projection_distance(left_plane,bin_top,centroid_model_new);
    //        bottom_dist = projection_distance(left_plane,bin_bottom,centroid_model_new);

    left_dist = bin_left(1) - centroid(1);
    right_dist = centroid(1) - bin_right(1);
    top_dist = bin_top(0) - centroid(0);
    bottom_dist = bin_bottom(0) - centroid(0);

    cout << "left distance: "  << left_dist << endl;
    cout << "right distance: "  << right_dist << endl;
    cout << "top distance: "  << top_dist << endl;
    cout << "bottom distance: "  << bottom_dist << endl;

    if(fabs(left_dist) < 0.08)
        return 0;
    if(fabs(right_dist) < 0.08)
        return 1;
    if(fabs(top_dist) < 0.08)
        return 2;
    if(fabs(bottom_dist) < 0.08)
        return 3;
    if(fabs(left_dist) > 0.1 & fabs(right_dist) > 0.1 && fabs(top_dist) > 0.1 && fabs(bottom_dist) > 0.1)
        return 4;

}

