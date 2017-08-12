#include<iitktcs_msgs_srvs/pose.h>
#include<3d_pose_estimtion.h>

#define r_angle 15
#define use_scene_centroid 1
#define deformable_offset 0.005
#define formable_offset 0
#define glass_height 0.045
#define region_space 0.07


EstimatePose::EstimatePose()
{
    //    getParameters();
    if(!nh.getParam("/ARC17_OBJECT_NAMES",model_names))
        ROS_ERROR("Problem in loading the model name parameters");
    model_fit = nh.advertiseService("/iitktcs/estimate_pose",&EstimatePose::callBack,this);
    nh.getParam("/bin_corners0",bin_corner0);
    nh.getParam("/bin_corners1",bin_corner1);
    tree = search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ> ());
    scene = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());
    input_scene = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>());

    vector<int> temp(K);
    pointIdxNKNSearch = temp;
}

EstimatePose::~EstimatePose()
{

}



bool EstimatePose::callBack(ensenso_testing::pose::Request &req, ensenso_testing::pose::Response &res)
{
    vector<ensenso_testing::objects_info> objects_info;

    sensor_msgs::PointCloud2 cloud_ros;

    for(int i=0;i<req.object_info.size();i++)
    {
        objects_info.push_back(req.object_info[i]);
        cout << "object ids: " << objects_info[i].id.data << endl;
    }
    cloud_ros = objects_info[0].roi;
    fromROSMsg(cloud_ros,*scene);
    object_id = objects_info[0].id.data -1;
    bin_id = req.bin_id.data;
    cout << "bin_id: " << bin_id << endl;
    cout << "object_id: " << object_id;
    modelName = model_names[object_id];
    cout << "Model Name: " << modelName << endl;

    ///////////Tranform PoinCloud in world frame ///////////////////

#if(TRANSFORM)
    transformPointCloud();
#endif


    if(modelName == "plastic_wine_glass")
    {
        cout << "special case algorithm for wine glass" << endl;
        specialCaseGlass();
        res.centroid.x = final_centroid(0); res.centroid.y = final_centroid(1); res.centroid.z = final_centroid(2);
        res.axis.x = final_axis(0); res.axis.y = final_axis(1); res.axis.z = final_axis(2);
        res.normal.x = final_normal(0); res.normal.y = final_normal(1); res.normal.z = final_normal(2);
        res.axis1.x = final_axis1(0); res.axis1.y = final_axis1(1); res.axis1.z = final_axis1(2);
        return true;
    }



    if(modelName == "mesh_cup")
    {
        cout << "special case algorithm for mesh cup" << endl;
        specialCaseMeshCup();
        res.centroid.x = final_centroid(0); res.centroid.y = final_centroid(1); res.centroid.z = final_centroid(2);
        res.axis.x = final_axis(0); res.axis.y = final_axis(1); res.axis.z = final_axis(2);
        res.normal.x = final_normal(0); res.normal.y = final_normal(1); res.normal.z = final_normal(2);
        res.axis1.x = final_axis1(0); res.axis1.y = final_axis1(1); res.axis1.z = final_axis1(2);
        return true;
    }


    ///////////get all the parameters from object yaml file ///////////
    getParameters();
    length_offset = dimensions[0];
    width_offset = dimensions[1];
    height_offset = dimensions[2];




    if(!scene->size()==0)
    {
        copyPointCloud(*scene,*input_scene);

        if(object_type == 0)
        {
            cout << "Rigid object" << endl;
            mainFuctionFormable();
        }

        else if(object_type == 1)
        {
            cout << "deformable object" << endl;
            mainFuctionDeFormable();
        }




        geometry_msgs::Pose mat_super4pcs_pose;
        //    Eigen::Affine3d p;
        tf::poseEigenToMsg(mat_super4pcs,mat_super4pcs_pose);

        //        if(modelName == "Bath_Sponge" )
        //        {
        //            cout << "centroid_offset" << centroid_offset << endl;
        //            final_centroid = final_centroid - final_normal*centroid_offset;
        //        }


        //        if(modelName == "Epsom_Salts")
        //        {
        //            cout << "centroid_offset" << centroid_offset << endl;
        //            final_centroid = final_centroid - final_normal*centroid_offset;
        //        }


        //        if(modelName == "Reynolds_Wrap")
        //        {
        //            cout << "centroid_offset" << centroid_offset << endl;
        //            final_centroid = final_centroid - final_normal*centroid_offset;
        //        }


        res.pose = mat_super4pcs_pose;
        res.centroid.x = final_centroid(0); res.centroid.y = final_centroid(1); res.centroid.z = final_centroid(2);
        res.axis.x = final_axis(0); res.axis.y = final_axis(1); res.axis.z = final_axis(2);
        res.normal.x = final_normal(0); res.normal.y = final_normal(1); res.normal.z = final_normal(2);
        res.axis1.x = final_axis1(0); res.axis1.y = final_axis1(1); res.axis1.z = final_axis1(2);
        res.angle.data = angle;

        objects_info.clear();
        ss_model_obj.str("");
        ss_scene_obj.str("");
        ss_model.str("");
        ss_super4pcs.str("");
        ss_super4pcs_matrix.str("");

    }
    else
    {
        cout << "Input cloud is empty" << endl;
        ss_model_obj.str("");
        ss_scene_obj.str("");
        ss_model.str("");
        ss_super4pcs.str("");
        ss_super4pcs_matrix.str("");
    }



    return true;

}


bool EstimatePose::mainFuctionFormable()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    double leafsize[3] = {0.02,0.02,.02};
    downSampleCloudVoxel(scene,scene_downsampled,leafsize);
    path = ros::package::getPath("iitktcs_pose_estimation");
    ss_scene_obj << path << "/data_new/" << modelName << "_scene.obj";
    createObjFile(scene_downsampled,ss_scene_obj.str());

    PointCloud<PointXYZ>::Ptr model(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr model_downsampled(new PointCloud<PointXYZ>());
    ss_model << path << "/pcd/" << modelName << ".pcd";
    pcl::io::loadPCDFile(ss_model.str().c_str(),*model);
    downSampleCloudVoxel(model,model_downsampled,leafsize);
    ss_model_obj << path <<"/data_new/" << modelName << ".obj";
    createObjFile(model_downsampled,ss_model_obj.str());

    double time = ros::Time::now().toSec();

    applySuper4pcs();
    cout << "Time taken in model fitting: " << ros::Time::now().toSec() << endl;
    PointCloud<PointXYZ>::Ptr model_transformed(new PointCloud<PointXYZ>());
    pcl::transformPointCloud(*model,*model_transformed,mat_super4pcs);




    //    pcl::visualization::PCLVisualizer vis("check_model_fit");
    //    vis.addPointCloud (model_transformed, ColorHandlerT (model_transformed, 0.0, 255.0, 0.0), "scene_allignment");
    //    vis.addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object");
    //    while(!vis.wasStopped())
    //        vis.spinOnce();
    //    vis.close();

    newcode(model_transformed);

    if(modelName == "ice_cube_tray")
    {
        cout << "special case for ice cube tray" << endl;
        specialCasetray(model_transformed);

    }
    if(modelName == "toilet_brush")
    {
        cout << "special case for Toilet Brush" << endl;
        specialCasebrush(model_transformed);

    }

    Eigen::Matrix4d mat;
    mat = mat_super4pcs.matrix();
    if(mat(0,0)==1 && mat(1,1)==1 && mat(2,2) ==1 && mat(3,3)==1)
    {
        cout << "Model is not fit going for deformable fuction" << endl;
        mainFuctionDeFormable();
    }




}

bool EstimatePose::mainFuctionDeFormable()
{
    vector<int> index;
    pcl::removeNaNFromPointCloud(*scene,*scene,index);

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(scene);

    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f Axis1 = eigen_vectors.col(0);
    Eigen::Vector3f Axis2 = eigen_vectors.col(1);

    Eigen::Vector4d centroid_compute;
    pcl::compute3DCentroid(*scene,centroid_compute);
    Eigen::Vector3f centroid;
    centroid(0) = centroid_compute(0);
    centroid(1) = centroid_compute(1);
    centroid(2) = centroid_compute(2);

    Eigen::Vector3f cent;
    searchKNN(centroid,cent);



    final_axis = Axis1;
    final_axis1 = Axis2;

    PointCloud<Normal>::Ptr scene_normals(new PointCloud<Normal>());
    computeNormal(scene,scene_normals);

    Eigen::Vector3f Normal;

    planFit(scene,scene_normals,Normal);
    //    if(primitive_type=="plane")
    //        planFit(scene,scene_normals,normal);
    //    else if (primitive_type=="cylinder")
    //        cylinderFit(scene,scene_normals,normal);
    //    else
    //        sphereFit(scene,scene_normals,normal);




    if(Normal(2)>0)
    {
        cout << "Normal remains same" << endl;
        Normal = Normal;
    }

    else
    {
        cout << "Inveting Normal" << endl;
        Normal = Normal*(-1);
    }


    pcl::visualization::PCLVisualizer::Ptr visu(new pcl::visualization::PCLVisualizer("deformable_object"));
    int case_input;
    case_input = getRegion(cent,visu);
    cout << "Case Input: " << case_input << endl;



    switch(case_input)
    {
    case 0:
    {
        cout << "In case 0, Near rack left region" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check < 0.25882)
        {
            Eigen::Matrix3f m;
            m << 0,0,1,
                    1,0,0,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << 1,0,0,
                    0,cos(ra),-sin(ra),
                    0,sin(ra),cos(ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);

            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;

    }
    case 1:
    {
        cout << "In case 1, Near rack right region" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check > -0.25882)
        {
            Eigen::Matrix3f m;
            m << 0,0,-1,
                    -1,0,0,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << 1,0,0,
                    0,cos(-ra),-sin(-ra),
                    0,sin(-ra),cos(-ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;
    }
    case 2:
    {
        cout << "In case 2 Near rack top region" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check > -0.25882)
        {
            Eigen::Matrix3f m;
            m << -1,0,0,
                    0,0,1,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << cos(-ra),0,sin(-ra),
                    0,       1,      0,
                    -sin(-ra),0,cos(-ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;

    }
    case 3:
    {
        cout << "In case 3, Near rack bottom region" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check < 0.25882)
        {
            Eigen::Matrix3f m;
            m << 1,0,0,
                    0,0,-1,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << cos(ra),0,sin(ra),
                    0,       1,      0,
                    sin(-ra),0,cos(ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;
    }
    case 4:
    {
        cout << "In case 4 Object lies in central region" << endl;
        break;
    }
    }

    final_normal = Normal;
    final_axis = Axis1;
    final_axis1 = Axis2;
    final_centroid = cent;

    cout << "Adjusting centroid value" << endl;
    cout << "centroid offset" << centroid_offset << endl;
    cent = cent - final_normal*(centroid_offset);
    final_centroid = cent - final_normal*(deformable_offset);
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
    ss_super4pcs << "time -p /home/manish/Super4PCS-master/build/Super4PCS -i " << ss_scene_obj.str().c_str() << " " << ss_model_obj.str().c_str() << " -o " << overlap << " -d 0.01 -t 1000 -n 100 -r super4pcs_fast.obj -m "<< path << "/data/mat_super4pcs_fast.txt";
    system(ss_super4pcs.str().c_str());

    ss_super4pcs_matrix << path << "/data/mat_super4pcs_fast.txt";
    ifstream input_file(ss_super4pcs_matrix.str().c_str());
    input_file.seekg(0,ios::end);
    input_file.seekg(20,ios::beg);

    vector<double> temp(16);

    for(int i=0;i<16;i++)
    {
        input_file >> temp[i];
        //        cout << temp[i] << endl;

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

    //    pcl::visualization::PCLVisualizer viewer("segmented cloud");
    //    viewer.addPointCloud(segmented_cloud);
    //    viewer.spin();
    //    viewer.close();


}


void EstimatePose::computeNormal(PointCloud<PointXYZ>::Ptr &scene, PointCloud<Normal>::Ptr &scene_normal)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (scene);
    ne.setKSearch (50);
    ne.compute (*scene_normal);


}


void EstimatePose::normalPoint(PointCloud<PointXYZ>::Ptr &scene,Eigen::Vector4f &plane_parameter)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (scene);
    ne.setRadiusSearch(0.015);
    //    ne.compute (*scene_normal);
    float curv;
    ne.computePointNormal(*scene,pointIdxNKNSearch,plane_parameter,curv);

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


    Normal.normalize();

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
    //    v->spin();
    //    v->close();

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
    //    v->spin();
    //    v->close();

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

    if(Normal(2) < 0 )
        Normal = Normal*(-1);


    //    std::cout << "values " <<  a  <<  " "  <<  b << " " << c << " " << d << endl;

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


    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*scene,cn);
    Eigen::Vector3f centroid;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);


    double color1[3] = {1,0,0};
    double color2[3] = {0,1,0};
    double color3[3] = {0,0,1};

    pcl::visualization::PCLVisualizer::Ptr visu ( new pcl::visualization::PCLVisualizer("object_deformable"));
    visu->addPointCloud (projetced_cloud, ColorHandlerT (projetced_cloud, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (object1, ColorHandlerT (object1, 0.0, 0.0, 255.0), "object_aligned");
    //    visu->addPointCloud(scene,ColorHandlerT(scene,255.0,0.0,0.0),"scene1");
    visu->addCoordinateSystem(0.5,0,0,0);
    addaxis(visu,Normal,centroid,color1,"Normal");
    addaxis(visu,final_axis,centroid,color2,"axis1");
    addaxis(visu,final_axis1,centroid,color3,"axis2");
    //    arrowvisulizer(object1,Normal,visu,id);
    //    arrowvisulizer(object1,final_axis,visu,id1);
    //    arrowvisulizer(object1,final_axis1,visu,"axis2");
//    visu->spin ();
//    visu->close();


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




void EstimatePose::newcode(pcl::PointCloud<pcl::PointXYZ>::Ptr &model)
{


    pcl::visualization::PCLVisualizer::Ptr visu(new pcl::visualization::PCLVisualizer("NormalAxis"));
    visu->addPointCloud (model, ColorHandlerT (model, 0.0, 255.0, 0.0), "scene");
    visu->addPointCloud (scene, ColorHandlerT (scene, 0.0, 0.0, 255.0), "object_aligned");
    visu->addCoordinateSystem(0.5,0,0,0);


    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*model,cn);
    Eigen::Vector3f centroid,centroid_temp;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);



    //    const std::string model_names[]=
    //    {
    //        "Toilet_Brush","Avery_Binder","Balloons","Band_Aid_Tape","Scotch_Sponges","Black_Fashion_Gloves","Burts_Bees_Baby_Wipes","Colgate_Toothbrush_4PK","Composition_Book","Crayons"
    //        ,"Duct_Tape","Epsom_Salts","Expo_Eraser","Fiskars_Scissors","Flashlight","Glue_Sticks","Hand_Weight","Hanes_Socks" ,"Hinged_Ruled_Index_Cards","Ice_Cube_Tray","Irish_Spring_Soap"
    //        ,"Laugh_Out_Loud_Jokes","Marbles","Measuring_Spoons","Mesh_Cup","Mouse_Traps","Pie_Plates","Plastic_Wine_Glass","Poland_Spring_Water","Reynolds_Wrap","Robots_DVD"
    //        ,"Robots_Everywhere","Bath_Sponge","Speed_Stick","White_Facecloth","Table_Cloth","Tennis_Ball_Container","Ticonderoga_Pencils","Tissue_Box","Windex"
    //    };

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(model);



    Eigen::Matrix3f eigen_vects = pca.getEigenVectors();
    Eigen::Vector3f major_axis = eigen_vects.col(0);
    Eigen::Vector3f minor_axis = eigen_vects.col(1);
    Eigen::Vector3f third_axis = eigen_vects.col(2);



    cout << "data Normal: " << data_normal[0] << " " << data_normal[1] << " " << data_normal[2] << endl;
    Eigen::Vector3f reference_axis(0,0,1);
    Eigen::Vector3f Normal,Axis1,Axis2;

    double dot_major = major_axis.dot(reference_axis);
    double dot_minor = minor_axis.dot(reference_axis);
    double dot_third = third_axis.dot(reference_axis);

    //    cout << "befor if else condition" << endl;
    /////////////*****Cuboidal Object*****///////////////////

    if(geometry_type == 0)
    {
        cout << "In case of cuboidal object" << endl ;
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

            else if(fabs(dot_third) > fabs(dot_minor) && fabs(dot_third) > fabs(dot_major))
            {
                if(dot_third > 0)
                    Normal = third_axis;
                else
                    Normal = third_axis*(-1);


                centroid_temp = centroid + (height_offset/2)*Normal;

                Axis1 = major_axis;
                Axis2 = minor_axis;

            }

            cout << "Normal in two axis axes" << Normal(0) << " " << Normal(1) << " " << Normal(2) << endl;

        }

        Eigen::Vector3f cent;

        Eigen::Vector4d new_cn;
        pcl::compute3DCentroid(*scene,new_cn);

#if(use_scene_centroid)
        Eigen::Vector3f scene_centroid;
        scene_centroid(0) = new_cn(0);
        scene_centroid(1) = new_cn(1);
        scene_centroid(2) = new_cn(2);

        searchKNN(scene_centroid,cent);
#else
        searchKNN(centroid_temp,cent);
#endif

        centroid_temp = cent;


    }


    /////////////*****Cylinder Object*****///////////////////


    //    else if(geometry_type == 1)
    //    {
    //        cout << "In case of cylinder object" << endl ;

    //        Eigen::Vector4d cn1;
    //        pcl::compute3DCentroid(*scene,cn1);
    //        centroid(0) = cn1(0); centroid(1) = cn1(1); centroid(2) = cn1(2);

    //        cout << "centroid befor KNN: " << centroid(0) << " " << centroid(1) << " " << centroid(2) << endl;
    //        searchKNN(centroid,centroid_temp);
    //        cout << "centroid after KNN: " << centroid_temp(0) << " " << centroid_temp(1) << " " << centroid_temp(2) << endl;

    //        //        pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
    //        //        computeNormal(scene,scene_normals);
    //        //        pcl::Normal no;
    //        tf::Vector3 avg_normal_temp;
    //        //        cout << "Size of computed points: " << pointIdxNKNSearch.size() << endl;
    //        //        for (int i = 0; i < pointIdxNKNSearch.size(); i++)
    //        //        {
    //        //            no = scene_normals->points[pointIdxNKNSearch[i]];
    //        //            tf::Vector3 temp_normal(no.normal[0], no.normal[1], no.normal[2]);

    //        //            avg_normal_temp += temp_normal;
    //        ////            cout << "temp_normal" << temp_normal.getX() << " "  << temp_normal.getY() << " " << temp_normal.getZ() << endl;
    //        //        }


    //        //        avg_normal_temp /= pointIdxNKNSearch.size();
    //        //        cout << "avg_Normal: " << avg_normal_temp.getX() << " " << avg_normal_temp.getY() << " " << avg_normal_temp.getZ() << endl;
    //        Eigen::Vector3f avg_normal;
    //        Eigen::Vector4f plane_param;
    //        normalPoint(scene,plane_param);

    //        avg_normal(0) = plane_param(0);
    //        avg_normal(1) = plane_param(1);
    //        avg_normal(2) = plane_param(2);





    //        //        avg_normal(0) = avg_normal_temp.getX();
    //        //        avg_normal(1) = avg_normal_temp.getY();
    //        //        avg_normal(2) = avg_normal_temp.getZ();

    //        double dot_m = major_axis.dot(reference_axis);
    //        double dot_n = avg_normal.dot(reference_axis);

    //        if(fabs(dot_m) > fabs(dot_n))
    //        {
    //            cout << "Normal is major axis " << endl;
    //            if(dot_m > 0)
    //                Normal = major_axis;

    //            else
    //                Normal = major_axis*(-1);

    //            Axis1 = avg_normal;
    //            Axis2 = minor_axis;
    //            //            centroid_temp = centroid + (length_offset/2)*Normal;
    //        }
    //        else
    //        {
    //            cout << "Normal is avg Normal : " << endl;
    //            double dot_r_m = avg_normal.dot(major_axis);
    //            double dot_r_n = avg_normal.dot(minor_axis);
    //            double dot_r_t = avg_normal.dot(third_axis);


    //            if(dot_n < 0 )
    //                Normal = avg_normal*(-1);
    //            else
    //                Normal = avg_normal;


    //            if(fabs(dot_r_m) > fabs(dot_r_n) && fabs(dot_r_m) > fabs(dot_r_t))
    //            {
    //                Axis1 = minor_axis;
    //                Axis2 = third_axis;

    //            }


    //            if(fabs(dot_r_n) > fabs(dot_r_m) && fabs(dot_r_n) > fabs(dot_r_t))
    //            {
    //                Axis1 = major_axis;
    //                Axis2 = third_axis;

    //            }


    //            if(fabs(dot_r_t) > fabs(dot_r_n) && fabs(dot_r_t) > fabs(dot_r_m))
    //            {
    //                Axis1 = minor_axis;
    //                Axis2 = major_axis;

    //            }


    //            //            centroid_temp = centroid + (height_offset/2)*Normal;
    //        }

    //        cout << "making normal axis perpandicular" << endl;

    //        Axis1 = Normal.cross(Axis2);
    //        Axis2 = Normal.cross(Axis1);

    //    }

    else if(geometry_type == 1)
    {
        pcl::PCA<pcl::PointXYZ> pca_scene;
        pca_scene.setInputCloud(scene);

        Eigen::Matrix3f cyliner_vects = pca_scene.getEigenVectors();
        Eigen::Vector3f cylinder_major = cyliner_vects.col(0);
        Eigen::Vector3f cylinder_minor = cyliner_vects.col(1);
        Eigen::Vector3f cylinder_third = cyliner_vects.col(2);

        Eigen::Vector4d new_cn;
        pcl::compute3DCentroid(*scene,new_cn);

        Eigen::Vector3f cent;
        centroid_temp(0) = new_cn(0); centroid_temp(1) = new_cn(1); centroid_temp(2) = new_cn(2);

        searchKNN(centroid_temp,cent);
        centroid_temp = cent;

        double p_major = cylinder_major.dot(reference_axis);
        double p_minor = cylinder_minor.dot(reference_axis);
        double p_third = cylinder_third.dot(reference_axis);

        if(fabs(p_major) > fabs(p_minor) && fabs(p_major) > fabs(p_third))
        {
            if(p_major > 0)
                Normal =  cylinder_major;
            else
                Normal = cylinder_major*(-1);

            Axis1 = cylinder_minor;
            Axis2 = cylinder_third;

        }

        if(fabs(p_minor) > fabs(p_third) && fabs(p_minor) > fabs(p_major))
        {
            if(p_minor > 0)
                Normal =  cylinder_minor;
            else
                Normal = cylinder_minor*(-1);

            Axis1 = cylinder_major;
            Axis2 = cylinder_third;

        }

        if(fabs(p_third) > fabs(p_minor) && fabs(p_third) > fabs(p_major))
        {
            if(p_third > 0)
                Normal =  cylinder_third;
            else
                Normal = cylinder_third*(-1);

            Axis1 = cylinder_major;
            Axis2 = cylinder_minor;

        }

        //       final_normal = Normal;
        //       final_centroid = centroid_temp;
        //       final_axis = Axis1;
        //       final_axis1 = Axis2;

    }



    cout << "centroid points after KNN: " << centroid_temp(0) << " " << centroid_temp(1) << " " << centroid_temp(2) << endl;
    vector<Eigen::Vector3f> sphere_points;
    sphere_points.push_back(centroid_temp);
    addsphere(visu,sphere_points,"sphere22");

    double color1[3] = {1,0,0};
    double color2[3] = {0,1,0};
    double color3[3] = {0,0,1};



    int case_input;
    case_input = getRegion(centroid,visu);
    cout << "Case Input: " << case_input << endl;


    switch(case_input)
    {
    case 0:
    {
        cout << "In case 0, Near rack left region" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check < 0.25882)
        {
            Eigen::Matrix3f m;
            m << 0,0,1,
                    1,0,0,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << 1,0,0,
                    0,cos(ra),-sin(ra),
                    0,sin(ra),cos(ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);

            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;

    }
    case 1:
    {
        cout << "In case 1, Near rack right region" << endl;
        double dot_check = top_plane.dot(Normal);
        if(dot_check > -0.25882)
        {
            Eigen::Matrix3f m;
            m << 0,0,-1,
                    -1,0,0,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << 1,0,0,
                    0,cos(-ra),-sin(-ra),
                    0,sin(-ra),cos(-ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;
    }
    case 2:
    {
        cout << "In case 2 Near rack top region" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check > -0.25882)
        {
            Eigen::Matrix3f m;
            m << -1,0,0,
                    0,0,1,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << cos(-ra),0,sin(-ra),
                    0,       1,      0,
                    -sin(-ra),0,cos(-ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;

    }
    case 3:
    {
        cout << "In case 3, Near rack bottom region" << endl;
        double dot_check = left_plane.dot(Normal);
        if(dot_check < 0.25882)
        {
            Eigen::Matrix3f m;
            m << 1,0,0,
                    0,0,-1,
                    0,1,0;

            double ra = (r_angle*M_PI)/180;
            Eigen::Matrix3f r;
            r << cos(ra),0,sin(ra),
                    0,       1,      0,
                    sin(-ra),0,cos(ra);

            Eigen::Matrix3f result;
            result = r*m;

            //            cout << "Resultant Matrix: " << result << endl;

            Normal = result.col(1);
            Axis1 =  result.col(0);
            Axis2 =  result.col(2);
            //            centroid_temp = centroid_temp + Normal*0.01;
        }
        break;
    }
    case 4:
    {
        cout << "In case 4 Object lies in central region" << endl;
        break;
    }
    }

    final_axis = Axis1;
    final_axis1 = Axis2;
    final_normal = Normal;
    Normal.normalize();
    Axis1.normalize();
    Axis2.normalize();

    addaxis(visu,final_normal,centroid,color1,"Normal");
    addaxis(visu,final_axis,centroid,color2,"Axis1");
    addaxis(visu,final_axis1,centroid,color3,"Axis2");

    double dot_normal = Normal.dot(reference_axis);

    angle = acos(dot_normal)*180/PI;
    cout << "angle with z axis: " << angle  << endl;

    centroid_temp = centroid_temp - final_normal*(centroid_offset);

    final_centroid = centroid_temp - final_normal*(formable_offset);

    ////calculating repose ///////////////////////
    //    reposeCalculation();
//    visu->spin();
//    visu->close();

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
    bin_centoid = (binA+binB+binC+binD)/4;

    vector<Eigen::Vector3f> sphere_points;
    sphere_points.push_back(binA);
    sphere_points.push_back(binB);
    sphere_points.push_back(binC);
    sphere_points.push_back(binD);

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

    if(fabs(left_dist) < region_space)
        return 0;
    if(fabs(right_dist) < region_space)
        return 1;
    if(fabs(top_dist) < region_space)
        return 2;
    if(fabs(bottom_dist) < region_space)
        return 3;
    if(fabs(left_dist) > region_space & fabs(right_dist) > region_space &&
            fabs(top_dist) > region_space && fabs(bottom_dist) > region_space)
        return 4;

}



void EstimatePose::getParameters()
{

    //    modelName = "Hand_Weight";
    const string param_geometry = "/" + modelName + "/geometry_type";
    if(!nh.getParam(param_geometry,geometry_type))
        ROS_ERROR("Problem in loading the geometry type parameters \n");


    cout << "object geometry parameter: " << geometry_type <<endl;

    const string param_object = "/" + modelName + "/object_type";
    if(!nh.getParam(param_object,object_type))
        ROS_ERROR("Problem in loading the object type parameters \n");

    cout << "object object parameter: " << object_type <<endl;

    const string param_overlap = "/" + modelName + "/overlap";
    if(!nh.getParam(param_overlap,overlap))
        ROS_ERROR("Problem in loading the overlap parameters \n");

    cout << "object overlap parameter: " << overlap <<endl;


    //    while(ros::ok())
    //    {
    //        if(data_normal.size() != 0)
    //            ros::shutdown();
    const string param_normal_data = "/" + modelName + "/normal_data";

    if(!nh.getParam(param_normal_data,data_normal))
        ROS_ERROR("Problem in loading the data normals parameters \n");


    //        cout << "Please Upload the parameter ------[meri phti hui hai] " << endl;

    //    }

    const string param_primitve_type = "/" + modelName + "/primitive_type";
    if(!nh.getParam(param_primitve_type,primitive_type))
        ROS_ERROR("Problem in loading the primitive type parameters \n");

    const string param_dimensions = "/" + modelName + "/dimensions";
    if(!nh.getParam(param_dimensions,dimensions))
        ROS_ERROR("Problem in loading the dimensions parameters \n");

    const string param_offset = "/" + modelName + "/offset";
    if(!nh.getParam(param_offset,centroid_offset))
        ROS_ERROR("Problem in loading the dimensions parameters \n");


}

//#if(TRANSFORM)
void EstimatePose::transformPointCloud()
{
    tf::StampedTransform transform;
    transform = getSensorToBaseFrame();
    Eigen::Affine3d pt_transform;



    tf::transformTFToEigen(transform,pt_transform);

    cout << "Transformation Matrix :" << pt_transform.matrix() << endl;
    pcl::transformPointCloud(*scene,*scene,pt_transform);

}
//#endif


void EstimatePose::searchKNN(Eigen::Vector3f &model_centroid,Eigen::Vector3f &scene_centroid)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (scene);
    pcl::PointXYZ searchPoint;
    //    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);


    searchPoint.x = model_centroid(0);
    searchPoint.y = model_centroid(1);
    searchPoint.z = model_centroid(2);

    //    cout << "centoid befor KNN: " << centroid_temp(0) << " " << centroid_temp(1) << " " << centroid_temp(2) << endl;

    double radius = 0.015;

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        //    if ( kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        cout << "find out the nearest neibhours : " << endl;
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            //            std::cout << "    "  <<   scene->points[ pointIdxNKNSearch[i] ].x
            //                      << " " << scene->points[ pointIdxNKNSearch[i] ].y
            //                      << " " << scene->points[ pointIdxNKNSearch[i] ].z << endl;
            //                                          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
    }

    Eigen::Vector3f cent(0,0,0);

    for (int i = 0; i < pointIdxNKNSearch.size (); i++)
    {
        cent(0) += scene->points[pointIdxNKNSearch[i]].x;
        cent(1) += scene->points[pointIdxNKNSearch[i]].y;
        cent(2) += scene->points[pointIdxNKNSearch[i]].z;
    }

    //    cout << "cent(0): " << cent(0) <<" " << cent(1) << " " << cent(2) << endl;
    scene_centroid(0) = cent(0)/pointIdxNKNSearch.size();
    scene_centroid(1) = cent(1)/pointIdxNKNSearch.size();
    scene_centroid(2) = cent(2)/pointIdxNKNSearch.size();

}
tf::StampedTransform  EstimatePose::getSensorToBaseFrame()
{
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;

    transform_listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(14));
    transform_listener.lookupTransform("/world", "/camera_link", ros::Time(0), transform);

    return transform;
}

void EstimatePose::reposeCalculation()
{
    cout << "calculate repose centroid, Axis,Normal" << endl;

    double length = 2*bellow_length*cos((180-angle)*M_PI/360);
    cout << "length of bellow to be moved" << length << endl;

    double offset = bellow_length - bellow_length*cos(angle*M_PI/180);
    final_repose_centroid(0) = final_centroid(0) + final_normal(0)*length;
    final_repose_centroid(1) = final_centroid(1) + final_normal(1)*length;
    final_repose_centroid(2) = final_centroid(2) + final_normal(2)*length - offset;


    Eigen::Vector3f befor_cent;

    befor_cent(0) = final_centroid(0) + final_normal(0)*before_normal;
    befor_cent(1) = final_centroid(1) + final_normal(1)*before_normal;
    befor_cent(2) = final_centroid(2) + final_normal(2)*before_normal;

    final_repose_axis(0) = befor_cent(0) - final_centroid(0);
    final_repose_axis(1) = befor_cent(1) - final_centroid(1);
    final_repose_axis(2) = 0;
    final_repose_axis.normalize();

    final_repose_normal(0) = 0;
    final_repose_normal(1) = 0;
    final_repose_normal(2) = 1;

}


void EstimatePose::specialCasetray(pcl::PointCloud<pcl::PointXYZ>::Ptr &model)
{
    cout  << "model is ice cube tray" << endl;

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(model);

    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f major_axis = eigen_vectors.col(0);
    Eigen::Vector3f minor_axis = eigen_vectors.col(1);
    Eigen::Vector3f third_axis = eigen_vectors.col(2);

    Eigen::Vector4d cn,cn1;
    pcl::compute3DCentroid(*model,cn);
    pcl::compute3DCentroid(*scene,cn1);



    Eigen::Vector3f centroid1,centroid2,scene_centroid;
    scene_centroid(0) = cn1(0);
    scene_centroid(1) = cn1(1);
    scene_centroid(2) = cn1(2);
    //    Eigen::Vector3f tmp_cn = centroid;
    Eigen::Vector3f tmp_cn_1;
    searchKNN(scene_centroid,tmp_cn_1);
    centroid1(0) = cn(0); centroid1(1) = cn(1); centroid1(2) = cn(2);

    centroid2 = centroid1;

    centroid1 = centroid1 + major_axis*(length_offset);
    centroid2 = centroid2 - major_axis*(length_offset);

    cout << "centroid1 " << centroid1(0)  << " " <<centroid1(1) << " " << centroid1(2) << endl;
    cout << "centroid2 " << centroid2(0)  << " " <<centroid2(1) << " " << centroid2(2) << endl;

    double distance_1,distance_2;

    distance_1 = calDistance(centroid1);
    distance_2 = calDistance(centroid2);

    cout << "distance 1 " << distance_1 << endl;
    cout << "distance 2 " << distance_2 << endl;
    Eigen::Vector3f cent;

    if(distance_1 > distance_2)
    {
        cout << "choosing centroid2 " << endl;
        searchKNN(centroid2,cent);
    }
    else
    {
        cout << "choosing centroid1" << endl;
        searchKNN(centroid1,cent);

    }


    if(cent(2) < tmp_cn_1(2))
    {
        cout << "Ice cube tray placed inverted" << endl;
        searchKNN(scene_centroid,cent);

        cent = cent +major_axis*(0.01);
        Eigen::Vector3f c1,c2;
        c1 = cent;
        c2 = cent;
        c1  = c1 + minor_axis*(0.02);
        c2  = c2 - minor_axis*(0.02);

        double d1 = calDistance(c1);
        double d2 = calDistance(c2);
        if(d1 > d2)
        {
            cent = c2;
        }
        else
            cent = c1;



    }
    else
        cout << "Ice cube tray placed front facing" << endl;


    final_centroid = cent - final_normal*(formable_offset);
    cout << "centroid before knn" << final_centroid(0)  << " " <<final_centroid(1) << " " << final_centroid(2) << endl;
}

void EstimatePose::specialCaseGlass()
{
    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*scene,cn);
    Eigen::Vector3f centroid;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);

    final_normal(0) = 0; final_normal(1) = 0; final_normal(2) = 1;
    final_axis(0) = 1; final_axis(1) = 0; final_axis(2) = 0;
    final_axis1(0) = 0; final_axis1(1) = 1; final_axis1(2) = 0;

    final_centroid = centroid + final_normal*(glass_height);


}

void EstimatePose::specialCaseMeshCup()
{
    cout << "special case for meshcup" << endl;
    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*scene,cn);

    pcl::visualization::PCLVisualizer vis("mesh_Cup");
    vis.addPointCloud(scene);
    vis.addCoordinateSystem(0.5,0,0,0);
//    vis.spin();
//    vis.close();

    Eigen::Vector3f centroid;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);

    Eigen::Vector3f cent;
    searchKNN(centroid,cent);

    final_normal(0) = 0; final_normal(1) = 0; final_normal(2) = 1;
    final_axis(0) = 1; final_axis(1) = 0; final_axis(2) = 0;
    final_axis1(0) = 0; final_axis1(1) = 1; final_axis1(2) = 0;

    final_centroid = cent - final_normal*(formable_offset);


}


void EstimatePose::specialCasebrush(PointCloud<PointXYZ>::Ptr &model)
{
    cout  << "model is toilet brush" << endl;

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(model);

    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
    Eigen::Vector3f major_axis = eigen_vectors.col(0);
    Eigen::Vector3f minor_axis = eigen_vectors.col(1);
    Eigen::Vector3f third_axis = eigen_vectors.col(2);

    Eigen::Vector4d cn;
    pcl::compute3DCentroid(*model,cn);

    Eigen::Vector3f centroid;
    centroid(0) = cn(0); centroid(1) = cn(1); centroid(2) = cn(2);

    cout << "centroid offset" << centroid_offset << endl;
    //        centroid_offset = 0.16;
    centroid = centroid + major_axis*(centroid_offset);

    Eigen::Vector3f cent;
    searchKNN(centroid,cent);
    final_centroid = cent - final_normal*(formable_offset);



    vector<Eigen::Vector3f> points;
    points.push_back(final_centroid);


    final_normal(0) = 0; final_normal(1) = 0; final_normal(2) = 1;
    final_axis(0) = 1; final_axis(1) = 0; final_axis(2) = 0;
    final_axis1(0) = 0; final_axis1(1) = 1; final_axis1(2) = 0;

    //        pcl::visualization::PCLVisualizer::Ptr visu ( new pcl::visualization::PCLVisualizer("Alignment"));
    //        visu->addPointCloud (model, ColorHandlerT (model, 0.0, 0.0, 255.0), "object_aligned");
    //        visu->addPointCloud(scene,ColorHandlerT(scene,255.0,0.0,0.0),"scene");
    //        visu->addCoordinateSystem(0.5,0,0,0);
    //        addsphere(visu,points,"spheres");
    //        visu->spin ();
    //        visu->close();


    //        final_centroid = cent - final_normal*(0.005);

}

double  EstimatePose::calDistance(Eigen::Vector3f &centroid)
{
    double distance = 0;

    cout  << "bin_centroid" << bin_centoid(0) << " "<< bin_centoid(1) << " "<< bin_centoid(2) << endl;

    for(int i=0;i<3;i++)
    {
        distance += ((centroid(i) - bin_centoid(i))*(centroid(i) - bin_centoid(i)));
    }

    distance = sqrt(distance);

    cout << "distance " << distance << endl;
    return distance;

}

//void alternateCalculation()
//{
//    cout << "model is not fit going for alternate code "<< endl;
//    Eigen::Vector4d cn;
//    pcl::compute3DCentroid(*scene,cn);
//    Eigen::Vector3f centroid;
//    centroid(0) = cn(0);
//    centroid(1) = cn(1);
//    centroid(2) = cn(2);

//    pcl::PCA<pcl::PointXYZ> pca;
//    pca.setInputCloud


//}


