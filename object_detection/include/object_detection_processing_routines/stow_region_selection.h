#ifndef STOW_REGION_SELECTION_H
#define STOW_REGION_SELECTION_H

void get_suction_or_gripping_status(int& prior_centroid, std::vector<Eigen::Matrix3f>& eigen_vectors,
                                    std::vector<Eigen::Vector3f>& eigen_values,
                                    std::vector<pcl::PointXYZRGB>& vector_centroids, bool& is_suction)
{
    const float max_gripper_openning = 0.06;

    is_suction = true;
    if ( .1 * std::sqrt(eigen_values[prior_centroid].y()) < max_gripper_openning)
        is_suction = false;
}

void get_prior_centroid(std::vector<pcl::PointXYZRGB>& vector_centroids,
                        int& prior_centroid)
{
    std::vector<float> depths;
    for (int i = 0; i < vector_centroids.size(); i++)
        depths.push_back(vector_centroids[i].z);

    float min_val = 1000000.0f;
    for (int i = 0; i < depths.size(); i++)
    {
        if (min_val > depths[i])
        {
            min_val = depths[i];
            prior_centroid = i;
        }
    }
}


void principle_components(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vector_ptr_clusters,
                          std::vector<pcl::PointXYZRGB>& vector_centroids,
                          std::vector<int>& vector_centroids_indices,
                          std::vector<Eigen::Matrix3f>& eigen_vectors, std::vector<Eigen::Vector3f>& eigen_values)
{
    for (int i = 0; i < vector_ptr_clusters.size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = 0;
        point.y = 0;
        point.z = 0;

        for (int j = 0; j < vector_ptr_clusters[i]->size(); j++)
        {
            point.x += vector_ptr_clusters[i]->at(j).x;
            point.y += vector_ptr_clusters[i]->at(j).y;
            point.z += vector_ptr_clusters[i]->at(j).z;
        }

        point.x /= vector_ptr_clusters[i]->size();
        point.y /= vector_ptr_clusters[i]->size();
        point.z /= vector_ptr_clusters[i]->size();

        pcl::search::KdTree<pcl::PointXYZRGB> kd_tree;
        kd_tree.setInputCloud(vector_ptr_clusters[i]);

        std::vector<int> indices;
        std::vector<float> distances;

        kd_tree.nearestKSearch(point, 1, indices, distances);

        vector_centroids.push_back(vector_ptr_clusters[i]->at(indices[0]));
        vector_centroids_indices.push_back(indices[0]);

        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(vector_ptr_clusters[i]);

        eigen_vectors.push_back(pca.getEigenVectors());
        eigen_values.push_back(pca.getEigenValues());
    }
}



void stow_region_growing_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
                                      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vector_ptr_clusters)
{


    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approx_voxel_grid;
    approx_voxel_grid.setLeafSize(0.003, 0.003, 0.003);
    approx_voxel_grid.setDownsampleAllData(true);
    approx_voxel_grid.setInputCloud(ptr_scene_cloud_RGB);
    approx_voxel_grid.filter(*ptr_scene_cloud_RGB);


    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud);


    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(ptr_scene_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    //pcl::

    std::cout << "SIZE = " << ptr_scene_cloud_RGB->size() << std::endl;

    /*
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> moving_least_square;
    moving_least_square.setInputCloud(ptr_scene_cloud_RGB);
    moving_least_square.setSearchRadius(0.01);
    moving_least_square.setPolynomialFit(true);
    moving_least_square.setPolynomialOrder(2);
    moving_least_square.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
    moving_least_square.setUpsamplingRadius(0.01);
    moving_least_square.setUpsamplingStepSize(0.004);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_smoothed_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    moving_least_square.process(*ptr_smoothed_scene_cloud_RGB);


    std::cout << "SIZE = " << ptr_scene_cloud_RGB->size() << std::endl;

    pcl::copyPointCloud(*ptr_smoothed_scene_cloud_RGB, *ptr_scene_cloud);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(ptr_scene_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    std::cout << "SMOOTHING DONE" << std::endl;
    */

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(20);
    reg.setInputCloud(ptr_scene_cloud);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(8.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    reg.setCurvatureTestFlag(true);
    reg.setSmoothModeFlag(true);

    std::cout << "REGION GROWING" << std::endl;

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    //	viewer.showCloud(ptr_scene_cloud); Sleep(10000);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_grown_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < clusters.size(); i++)
    {
        vector_ptr_clusters.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr());
        vector_ptr_clusters[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int j = 0; j < clusters.at(i).indices.size(); j++)
            vector_ptr_clusters[i]->push_back(ptr_scene_cloud_RGB->at(clusters.at(i).indices.at(j)));

        *ptr_grown_scene_cloud_RGB += *vector_ptr_clusters[i];
    }


    /*
    while (counter < clusters[0].indices.size())
    {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
    std::cout << std::endl;
    }
    */

    std::cout << "CLUSTERING DONE" << std::endl;


}



void stow_region_growing_segmentation_RGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB)
{

    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approx_voxel_grid;
    approx_voxel_grid.setLeafSize(0.003, 0.003, 0.003);
    approx_voxel_grid.setDownsampleAllData(true);
    approx_voxel_grid.setInputCloud(ptr_scene_cloud_RGB);
    approx_voxel_grid.filter(*ptr_scene_cloud_RGB);


    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(ptr_scene_cloud_RGB);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);


    pcl::RegionGrowingRGB<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(60);
    reg.setMaxClusterSize(20000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(ptr_scene_cloud_RGB);
    reg.setRegionColorThreshold(5);
    reg.setPointColorThreshold(5);

    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(15.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    pcl::visualization::CloudViewer viewer("Cluster viewer");
    //viewer.showCloud(ptr_scene_cloud); Sleep(10000);

    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
                 std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;


    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vector_ptr_clouds;

    for (int i = 0; i < clusters.size(); i++)
    {
        vector_ptr_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr());
        vector_ptr_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int j = 0; j < clusters.at(i).indices.size(); j++)
            vector_ptr_clouds[i]->push_back(ptr_scene_cloud_RGB->at(clusters.at(i).indices.at(j)));
    }

    /*
    while (counter < clusters[0].indices.size())
    {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
    std::cout << std::endl;
    }
    */

    std::cout << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    while (!viewer.wasStopped())
    {
        viewer.showCloud(colored_cloud);  std::cout << "COLORED CLOUD" << std::endl; sleep(1);
        viewer.showCloud(ptr_scene_cloud_RGB); std::cout << "PTR SCENE CLOUD" << std::endl; sleep(1);
        //	for (int i = 0; i < vector_ptr_clouds.size(); i++)
        {
            //		viewer.showCloud(vector_ptr_clouds[i]);  std::cout << "CLUSTERS CLOUD = " << i <<  std::endl; Sleep(1000);
        }
    }
}


bool select_stowing_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_tote_center_cloud_RGB,
                            pcl::PointXYZRGB& centroid_3d,
                            Eigen::Matrix3f& stow_eigen_vectors,
                            pcl::Normal& normal, bool& is_stow_suction,
                            int& direction,
                            int& centroid_index_2d)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vector_ptr_clusters;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_voxel_tote_center_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*ptr_tote_center_cloud_RGB,*ptr_voxel_tote_center_cloud_RGB);

    stow_region_growing_segmentation(ptr_voxel_tote_center_cloud_RGB, vector_ptr_clusters);

    std::vector<Eigen::Matrix3f> eigen_vectors;
    std::vector<Eigen::Vector3f> eigen_values;
    std::vector<pcl::PointXYZRGB> centroids;
    std::vector<int> vector_centroids_indices;

    std::cout << "GETTING PRINCIPLE COMPONENTS" << std::endl;

    principle_components(vector_ptr_clusters, centroids,
                         vector_centroids_indices,
                         eigen_vectors, eigen_values);

    std::cout << "PRINCIPLE COMPONENTS DONE" << std::endl;

    int prior_centroid;
    bool is_suction;


    std::cout << "GETTING PRINCIPLE CENTROID" << std::endl;
    get_prior_centroid(centroids, prior_centroid);

    std::cout << "GETTING SUCTION OR GRIPPING" << std::endl;
    get_suction_or_gripping_status(prior_centroid, eigen_vectors, eigen_values, centroids, is_suction);

    std::cout << "EIGEN VALUES  = ";
    for (int i = 0; i < 3; i++)
        std::cout << 0.1 *std::sqrt(eigen_values[prior_centroid][i]) << "  ";
    std::cout << std::endl;

    std::cout << "EIGEN VECTORS = " << eigen_vectors[prior_centroid] << std::endl << std::endl;

    std::cout << " PRIOR CLOUD NO = " << prior_centroid << std::endl;

    if (is_suction)
        std::cout << " SUCTION" << std::endl;
    else
        std::cout << " GRIPPING" << std::endl;

    std::cout << "EIGEN VALUE 2nd" << 2 * eigen_values[prior_centroid][1] << std::endl;

    std::cout << "visualizing" <<std::endl;
    pcl::visualization::CloudViewer viewer("Cluster viewer");


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_with_axis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*vector_ptr_clusters[prior_centroid],*ptr_with_axis_cloud);


    pcl::PointXYZRGB point;
    point.r =255;
    point.g =0;
    point.b =0;

    for(int i=0;i<100;i++)
    {
        point.x  = centroids[prior_centroid].x + i* 0.001f * eigen_vectors[prior_centroid](0,0);
        point.y  = centroids[prior_centroid].y + i* 0.001f * eigen_vectors[prior_centroid](1,0);
        point.z  = centroids[prior_centroid].z + i* 0.001f * eigen_vectors[prior_centroid](2,0);
        ptr_with_axis_cloud->push_back(point);
    }

    point.r =0;
    point.g =255;
    point.b =0;

    for(int i=0;i<100;i++)
    {
        point.x  = centroids[prior_centroid].x + i* 0.001f * eigen_vectors[prior_centroid](0,1);
        point.y  = centroids[prior_centroid].y + i* 0.001f * eigen_vectors[prior_centroid](1,1);
        point.z  = centroids[prior_centroid].z + i* 0.001f * eigen_vectors[prior_centroid](2,1);
        ptr_with_axis_cloud->push_back(point);
    }

    point.r =0;
    point.g =0;
    point.b =255;

    for(int i=0;i<100;i++)
    {
        point.x  = centroids[prior_centroid].x + i* 0.001f * eigen_vectors[prior_centroid](0,2);
        point.y  = centroids[prior_centroid].y + i* 0.001f * eigen_vectors[prior_centroid](1,2);
        point.z  = centroids[prior_centroid].z + i* 0.001f * eigen_vectors[prior_centroid](2,2);
        ptr_with_axis_cloud->push_back(point);
    }


    int display_loop = 0;
    while (display_loop++ < 8) // !viewer.wasStopped())
    {
        viewer.showCloud(ptr_with_axis_cloud);  std::cout << "PRIOR CLOUD RGB" << std::endl; sleep(1);
        viewer.showCloud(ptr_voxel_tote_center_cloud_RGB);  std::cout << "REMAINING CLOUD RGB" << std::endl; sleep(1);
    }


    std::cout << "FILLING DATA" <<std::endl;

    centroid_3d  =  centroids[prior_centroid];

    std::cout << "FILLING EIGENS" <<std::endl;

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            stow_eigen_vectors(i,j) = eigen_vectors[prior_centroid](i,j);

    std::cout << "STOW" <<std::endl;

    is_stow_suction = is_suction;
    direction =5;

    std::cout << "INDICES SIZE = " << vector_centroids_indices.size() << std::endl;
    std::cout << "INDICES = " << vector_centroids_indices[prior_centroid] << std::endl;
    std::vector<int> indices;
    indices.push_back(vector_centroids_indices[prior_centroid]);


    std::cout << "COMPUTING NORMAL" <<std::endl;


    std::cout << "SEARCHING KNN" <<std::endl;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ptr_kd_tree->setInputCloud(ptr_tote_center_cloud_RGB);

    std::vector<int> vector_indices;
    std::vector<float> vector_distances;
    ptr_kd_tree->nearestKSearch(centroid_3d,1,vector_indices,vector_distances);

    std::cout << "SEARCHING DONE and INDEX = " << vector_indices[0] << std::endl;
    centroid_index_2d = vector_indices[0];

    float curvature;
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> normal_estimator;
    normal_estimator.setKSearch(30);
    normal_estimator.setInputCloud(ptr_tote_center_cloud_RGB);
    normal_estimator.computePointNormal(*ptr_tote_center_cloud_RGB,indices,
                                        normal.normal[0],normal.normal[1],normal.normal[2],curvature);


    normal.normal[0] = stow_eigen_vectors(0,2);
    normal.normal[1] = stow_eigen_vectors(1,2);
    normal.normal[2] = stow_eigen_vectors(2,2);



    return true;
}


#endif // STOW_REGION_SELECTION_H
