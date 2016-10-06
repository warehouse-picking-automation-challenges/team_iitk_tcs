#ifndef CLOUD_REGISTRATION_H
#define CLOUD_REGISTRATION_H

//----- REMOVE FRONT SIDE TOP and BACK PLANES

void remove_bin_boundaries(SHELF_DETAILS& shelf_details,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, int bin_no,
    Eigen::Affine3f& transformation_scene_to_model)
{
    transformation_scene_to_model(0, 3) -= shelf_details.ptr_centroid_cloud->at(bin_no).x;
    transformation_scene_to_model(1, 3) -= shelf_details.ptr_centroid_cloud->at(bin_no).y;

    pcl::transformPointCloud(*ptr_scene_cloud_RGB, *ptr_bin_cloud_RGB, transformation_scene_to_model);

    float field_limits[6];
    field_limits[0] = -shelf_details.bin_widths_heights[bin_no * 2] / 2.0f + SIDE_WALL_REMOVAL_ERROR;
    field_limits[1] =  shelf_details.bin_widths_heights[bin_no * 2] / 2.0f - SIDE_WALL_REMOVAL_ERROR;
    //field_limits[2] = -shelf_details.bin_widths_heights[bin_no * 2 + 1] + SIDE_WALL_REMOVAL_ERROR;
    field_limits[2] = -shelf_details.bin_widths_heights[bin_no * 2 + 1] / 2.0f - 0.03;
    field_limits[3] =  shelf_details.bin_widths_heights[bin_no * 2 + 1] / 2.0f- SIDE_WALL_REMOVAL_ERROR;
    field_limits[4] = DEPTH_WALL_REMOVAL_ERROR;
    field_limits[5] = shelf_details.bin_depth - DEPTH_WALL_REMOVAL_ERROR;

    filter_point_cloud(ptr_bin_cloud_RGB, ptr_bin_cloud_RGB, ptr_pixel_cloud_RGB,
      true, false, false, NULL, field_limits, 1.4, 10);


//    pcl::visualization::CloudViewer cloud_viewer("CLOUD");
//    while (1)
//    {
//        cloud_viewer.showCloud(ptr_scene_cloud_RGB); sleep(1);
//        cloud_viewer.showCloud(ptr_bin_cloud_RGB); sleep(1);
//    }

}


//----ICP REGISTRATION

bool icp_incremental_two_cloud_aligning(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned, Eigen::Affine3f& transformation)
{

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setInputSource(source);
    icp.setInputTarget(target);

    icp.setEuclideanFitnessEpsilon(.03);
    icp.setMaximumIterations(2);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-11);


    Eigen::Matrix4f prev, Ti;
    pcl::copyPointCloud(*source, *aligned);

    Ti = Eigen::Matrix4f::Identity();

    for (int i = 0; i < 40; ++i)
    {
        printf("Iteration Nr. %d.\n", i);

        // Estimate
        icp.setInputSource(aligned);
        icp.align(*aligned);

        //accumulate transformation between each Iteration
        Ti = icp.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon())
            icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.01);

        prev = icp.getLastIncrementalTransformation();

        // visualize current state

    }

    //	txform_mtx = txform_mtx.inverse();

    //pcl::copyPointCloud(*source, *aligned);
    //pcl::transformPointCloud(*aligned, *aligned, txform_mtx);

    bool has_converged = icp.hasConverged();

    cout << has_converged << endl;
    cout << icp.getFinalTransformation() << endl;
    cout << Ti << endl;

    transformation = Ti;

    //---ICP Normals Done
    return has_converged;
}

bool icp_nonlinear_incremental_two_cloud_aligning(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned, Eigen::Affine3f& transformation)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr aligned_with_normal(new pcl::PointCloud<pcl::PointNormal>);


    pcl::IterativeClosestPointNonLinear< pcl::PointNormal, pcl::PointNormal> icp_normal;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    normal_estimator.setSearchMethod(kdtree);
    normal_estimator.setKSearch(50);

    normal_estimator.setInputCloud(source);
    normal_estimator.compute(*source_with_normal);
    pcl::copyPointCloud(*source, *source_with_normal);

    normal_estimator.setInputCloud(target);
    normal_estimator.compute(*target_with_normal);
    pcl::copyPointCloud(*target, *target_with_normal);


    icp_normal.setInputSource(source_with_normal);
    icp_normal.setInputTarget(target_with_normal);


    icp_normal.setMaxCorrespondenceDistance(0.1);
    icp_normal.setMaximumIterations(2);
    icp_normal.setTransformationEpsilon(1e-6);


    Eigen::Matrix4f prev, prev_final, Ti;
    pcl::copyPointCloud(*source_with_normal, *aligned_with_normal);


    Ti = Eigen::Matrix4f::Identity();
    prev_final = Eigen::Matrix4f::Zero();

    for (int i = 0; i <40; ++i)
    {
        PCL_INFO("Iteration Nr. %d.\n", i);

        // Estimate
        icp_normal.setInputSource(aligned_with_normal);
        icp_normal.align(*aligned_with_normal);

        //accumulate transformation between each Iteration
        Ti = icp_normal.getFinalTransformation() * Ti;
        cout << Ti << endl;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((icp_normal.getLastIncrementalTransformation() - prev).sum()) < icp_normal.getTransformationEpsilon())
            icp_normal.setMaxCorrespondenceDistance(icp_normal.getMaxCorrespondenceDistance() - 0.001);

        prev = icp_normal.getLastIncrementalTransformation();

        if (fabs((icp_normal.getFinalTransformation() - prev_final).sum()) < 1e-9)
            break;

        prev_final = icp_normal.getFinalTransformation();
        // visualize current state
    }

    //pcl::transformPointCloudWithNormals(*aligned_with_normal, *aligned_with_normal, txform_mtx);
    //pcl::copyPointCloud(*source, *aligned);

    //pcl::transformPointCloudWithNormals(*aligned_with_normal, *aligned_with_normal, txform_mtx);
    pcl::copyPointCloud(*aligned_with_normal, *aligned);

    bool has_converged = icp_normal.hasConverged();

    cout << has_converged << endl;
    cout << icp_normal.getFinalTransformation() << endl;
    cout << Ti << endl;

    transformation = Ti;

    //---ICP Normals Done
    return has_converged;
}

void register_shelf(SHELF_DETAILS& shelf_details,pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud, Eigen::Affine3f& transformation)
{
    float field_limits_bin_cutting[6];
    field_limits_bin_cutting[0] = -BIN_CUTTING_CENTROID_ERROR;
    field_limits_bin_cutting[1] = shelf_details.rack_inner_width_height[0] + BIN_CUTTING_CENTROID_ERROR;
    field_limits_bin_cutting[2] = -BIN_CUTTING_CENTROID_ERROR;
    field_limits_bin_cutting[3] = shelf_details.rack_inner_width_height[1] + BIN_CUTTING_CENTROID_ERROR;
    field_limits_bin_cutting[4] = -BIN_CUTTING_DEPTH_ERROR;
    field_limits_bin_cutting[5] = BIN_CUTTING_DEPTH_ERROR;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cropped_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_pixel_cloud_RGB(NULL);

    filter_point_cloud(ptr_scene_cloud, ptr_cropped_scene_cloud, ptr_pixel_cloud_RGB,
        true, false, false, NULL, field_limits_bin_cutting, 1.4, 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //Eigen::Affine3f transformation;
    //icp_nonlinear_incremental_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);
    icp_incremental_two_cloud_aligning(ptr_cropped_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud,transformation);

/*
    pcl::visualization::CloudViewer cloud_viewer("Cloud");
        while (1)
    {
    cloud_viewer.showCloud(ptr_scene_cloud); sleep(1);
    cloud_viewer.showCloud(ptr_cropped_scene_cloud); sleep(1);
    cloud_viewer.showCloud(ptr_aligned_cloud); sleep(1);
    cloud_viewer.showCloud(ptr_bin_cloud); sleep(1);
        }
*/

    //icp_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);
    //pcl::transformPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud_RGB, transformation);
}

void compute_corresponding_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_bin_points,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_transformed_bin_points)
{
    //------   LEFT BOTTOM - LEFT UPPER - RIGHT UPPER - RIGHT BOTTOM

    //int LB = 0;
    //int LU = 1;
    //int RU = 2;
    //int RB = 3;
    int max_points = ptr_scene_bin_points->size();
    float distance[4];

    for (int i = 0; i < max_points; i++)
    {
        int index_point1 = i;
        int index_point2 = (i + 1) % max_points;

        Eigen::Vector3f point1;
        Eigen::Vector3f point2;

        point1.x() = ptr_scene_bin_points->at(index_point1).x;
        point1.y() = ptr_scene_bin_points->at(index_point1).y;
        point1.z() = ptr_scene_bin_points->at(index_point1).z;

        point2.x() = ptr_scene_bin_points->at(index_point2).x;
        point2.y() = ptr_scene_bin_points->at(index_point2).y;
        point2.z() = ptr_scene_bin_points->at(index_point2).z;

        distance[i] = (point1 - point2).norm();
        printf("%f \n ", distance[i]);
    }

    ptr_transformed_bin_points->clear();

    pcl::PointXYZ point;

    point.x = 0.0f;
    point.y = 0.0f;
    point.z = 0.0f;
    ptr_transformed_bin_points->push_back(point);

    point.x = 0.0f;
    point.y = distance[0];
    ptr_transformed_bin_points->push_back(point);

    point.x = distance[1];
    point.y = distance[2];
    ptr_transformed_bin_points->push_back(point);

    point.x = distance[3];
    point.y = 0.0f;
    ptr_transformed_bin_points->push_back(point);

}

void get_transformation_matrix_from_scene_to_origin(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_bin_points,
    Eigen::Affine3f& transformation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed_bin_points(new pcl::PointCloud<pcl::PointXYZ>);
    compute_corresponding_points(ptr_scene_bin_points, ptr_transformed_bin_points);

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    transformation_estimation.estimateRigidTransformation(*ptr_scene_bin_points, *ptr_transformed_bin_points, transformation.matrix());
}

void get_transformation_from_scene_to_model(SHELF_DETAILS& shelf_details,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_hough_points_cloud,
    Eigen::Affine3f& transformation_scene_to_model,std::string bin_file_path)
{
    Eigen::Affine3f transformation_inverted_cloud = Eigen::Affine3f::Identity();
    transformation_inverted_cloud(1, 1) = -1;   // INVERT Y COORDINATE

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_temp_hough_points_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*ptr_scene_cloud, *ptr_scene_cloud, transformation_inverted_cloud);
    pcl::transformPointCloud(*ptr_hough_points_cloud, *ptr_temp_hough_points_cloud, transformation_inverted_cloud);

    Eigen::Affine3f transformation_scene_to_origin;
    get_transformation_matrix_from_scene_to_origin(ptr_temp_hough_points_cloud, transformation_scene_to_origin);

    pcl::transformPointCloud(*ptr_scene_cloud, *ptr_scene_cloud, transformation_scene_to_origin);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);



    std::stringstream bin_file_name;
    bin_file_name << bin_file_path.c_str() ;
    std::cout << "Bin file name \t" << bin_file_path.c_str() << endl;
    pcl::io::loadPCDFile(bin_file_name.str().c_str(), *ptr_bin_cloud);
    //pcl::io::loadPLYFile(bin_file_name.str().c_str(), *ptr_bin_cloud);

    //pcl::visualization::CloudViewer cloud_viewer("cloud");
    //while(1)cloud_viewer.showCloud(ptr_bin_cloud);


    Eigen::Affine3f transformation_origin_to_model;
    register_shelf(shelf_details, ptr_scene_cloud, ptr_bin_cloud, transformation_origin_to_model);

    transformation_scene_to_model = Eigen::Affine3f::Identity();
    transformation_scene_to_model = transformation_inverted_cloud * transformation_scene_to_model;
    transformation_scene_to_model = transformation_scene_to_origin * transformation_scene_to_model;
    transformation_scene_to_model = transformation_origin_to_model *transformation_scene_to_model;
    transformation_scene_to_model(0, 3) -= shelf_details.model_origin_offset_from_scene[0];
    transformation_scene_to_model(1, 3) -= shelf_details.model_origin_offset_from_scene[1];


    std::cout << transformation_inverted_cloud.matrix() << std::endl << std::endl;
    std::cout << transformation_scene_to_origin.matrix() << std::endl << std::endl;
    std::cout << transformation_origin_to_model.matrix() << std::endl << std::endl;
    std::cout << transformation_scene_to_model.matrix() << std::endl << std::endl;
}


#endif // CLOUD_REGISTRATION_H
