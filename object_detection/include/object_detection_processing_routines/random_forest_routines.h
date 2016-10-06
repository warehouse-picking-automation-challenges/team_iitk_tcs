#ifndef RANDOM_FOREST_ROUTINES_H
#define RANDOM_FOREST_ROUTINES_H


void get_all_clouds_and_kd_trees(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
    cv::Mat& cloud_color_image,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& ptr_scene_cloud_I,
    pcl::PointCloud<pcl::PointNormal>::Ptr& ptr_scene_normal_cloud,
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr& ptr_kd_tree_RGB,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr& ptr_kd_tree,
    pcl::search::KdTree<pcl::PointXYZI>::Ptr& ptr_kd_tree_I)
{
    pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud);
    pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud_I);

    cloud_to_image(ptr_scene_cloud_RGB, cloud_color_image);
    cv::Mat gray_image;
    cv::cvtColor(cloud_color_image, gray_image, CV_BGR2GRAY);

    for (int i = 0; i < ptr_scene_cloud_I->size(); i++)
        ptr_scene_cloud_I->at(i).intensity = gray_image.at<float>(i);

    ptr_kd_tree_RGB->setInputCloud(ptr_scene_cloud_RGB);
    ptr_kd_tree->setInputCloud(ptr_scene_cloud);
    ptr_kd_tree_I->setInputCloud(ptr_scene_cloud_I);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

    normal_estimator.setRadiusSearch(0.015);
    normal_estimator.setSearchMethod(ptr_kd_tree);
    normal_estimator.setInputCloud(ptr_scene_cloud);
    normal_estimator.compute(*ptr_scene_normal_cloud);

}


//--COMBINING FEATURES

void combine_all_feature(cv::Mat& mask, std::vector<cv::Mat>& vector_features, cv::Mat& combined_feature)
{
    int total_columns = 0;
    for (int i = 0; i < vector_features.size(); i++)
        total_columns += vector_features[i].cols;

    int rows = 0;
    for (int i = 0; i < mask.rows; i++)
        if (mask.at<unsigned char>(i))rows++;

    printf("ROWS COLS %d  %d\n", rows, total_columns);

    combined_feature.release();
    combined_feature.create(rows, total_columns, CV_32FC1);

    rows = 0;
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask.at<unsigned char>(i))
        {
            int col = 0;
            float * pixel = (float*)(combined_feature.data + rows * combined_feature.step[0]);
            for (int j = 0; j < vector_features.size(); j++)
            {
                float * feature_pixel = (float*)(vector_features[j].data + i * vector_features[j].step[0]);

                for (int k = 0; k < vector_features[j].cols; k++)
                    pixel[col++] = feature_pixel[k];
            }
            rows++;
        }
    }
}

void get_combined_histogram_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& ptr_scene_cloud_I,
    pcl::PointCloud<pcl::PointNormal>::Ptr& ptr_scene_normal_cloud,
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr& ptr_kd_tree_RGB,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr& ptr_kd_tree,
    pcl::search::KdTree<pcl::PointXYZI>::Ptr& ptr_kd_tree_I,
    cv::Mat& combined_feature, cv::Mat& mask, std::vector<int>& all_point_indices)
{
    std::vector<cv::Mat> vector_features;
    std::vector<cv::Mat> vector_masks;

    mask.create(all_point_indices.size(), 1, CV_8UC1);
    mask = 1;

    int feature_no = 0;

    vector_features.push_back(cv::Mat());
    vector_masks.push_back(cv::Mat());
    compute_depth_mean_var_histogram_feature_gaussian_partition(ptr_kd_tree,
        ptr_scene_cloud, ptr_scene_normal_cloud, vector_features[feature_no], vector_masks[feature_no], all_point_indices);

    feature_no++;
    vector_features.push_back(cv::Mat());
    vector_masks.push_back(cv::Mat());
    compute_gradient_histogram_feature(ptr_kd_tree_I, ptr_scene_cloud_I, ptr_scene_normal_cloud,
        vector_features[feature_no], vector_masks[feature_no], all_point_indices);

    feature_no++;
    vector_features.push_back(cv::Mat());
    compute_color_histogram_feature(ptr_kd_tree_RGB, ptr_scene_cloud_RGB, vector_features[feature_no], all_point_indices);

    for (int i = 0; i < vector_masks.size(); i++)
        cv::multiply(vector_masks[i], mask, mask);

    combine_all_feature(mask, vector_features, combined_feature);
}

void get_random_forest_posterior(std::vector<int>& bin_members,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
                                  cv::Mat& random_forest_posterior,std::string& database_path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_bin_voxel_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_bin_cloud_I(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointNormal>::Ptr ptr_normals_bin_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ptr_kd_tree_RGB(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr ptr_kd_tree_I(new pcl::search::KdTree<pcl::PointXYZI>);

    cv::Mat cloud_color_image;

    get_all_clouds_and_kd_trees(ptr_bin_cloud_RGB, cloud_color_image, ptr_bin_cloud, ptr_bin_cloud_I,
        ptr_normals_bin_cloud, ptr_kd_tree_RGB, ptr_kd_tree, ptr_kd_tree_I);

    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setLeafSize(0.004, 0.004, 0.004);
    voxel_filter.setDownsampleAllData(true);
    voxel_filter.setInputCloud(ptr_bin_cloud_RGB);
    voxel_filter.filter(*ptr_bin_voxel_cloud_RGB);

    std::vector<int> all_point_indices;

    for (int i = 0; i < ptr_bin_voxel_cloud_RGB->size(); i++)
    {
        std::vector<int> indices;
        std::vector<float> distances;

        ptr_kd_tree_RGB->nearestKSearch(ptr_bin_voxel_cloud_RGB->at(i), 1, indices, distances);
        all_point_indices.push_back(indices[0]);
    }

    cv::Mat combined_feature;
    cv::Mat mask;

    std::cout << "GETTING RANDOM FOREST FEATURES" << std::endl;

    get_combined_histogram_features(ptr_bin_cloud_RGB, ptr_bin_cloud, ptr_bin_cloud_I, ptr_normals_bin_cloud,
        ptr_kd_tree_RGB, ptr_kd_tree, ptr_kd_tree_I, combined_feature,mask, all_point_indices);

    std::cout << "GOT RANDOM FOREST FEATURES" << std::endl;

    int valid_points = 0;
    for (int i = 0; i < mask.rows; i++)
        if (mask.at<unsigned char>(i))valid_points++;

    std::cout << "VALID POINTS = " << valid_points << "  " << std::endl;

    cv::Mat posterior(valid_points, 1, CV_32FC1);

    int model_no = bin_members[0];

    std::stringstream trained_random_forest_file_name;
    trained_random_forest_file_name << database_path << "/image_database/trained random forest/RANDOM_FOREST_";
    trained_random_forest_file_name << model_names[model_no] << ".xml";

    std::cout << trained_random_forest_file_name.str() << std::endl;

    cv::Ptr<cv::ml::RTrees> random_forest;
    random_forest = cv::Algorithm::load<cv::ml::RTrees>(trained_random_forest_file_name.str());
    random_forest->predict(combined_feature, posterior, cv::ml::RTrees::PREDICT_AUTO);

    std::cout << "RANDOM FOREST PREDICTION DONE" << std::endl;

//    for (int i = 0; i < posterior.rows; i++)
//    {
//        if (posterior.at<float>(i) > 0.5)
//            printf("%f ", posterior.at<float>(i));
//    }
//    printf("\n");

    random_forest_posterior.release();
    random_forest_posterior.create(ptr_bin_cloud_RGB->size(),1,CV_32FC1);

    for(int i=0;i<random_forest_posterior.rows;i++)
        random_forest_posterior.at<float>(i) =0.0f;

    valid_points = 0;
    for (int i = 0; i < all_point_indices.size(); i++)
    {
    //std::cout << i << "  ";
        if (mask.at<unsigned char>(i))
        {
            pcl::PointIndices point_indices;
            std::vector<float>  distances;

            float radii = 0.015;
            int point_index = all_point_indices[i];

            ptr_kd_tree->radiusSearch(*ptr_bin_cloud, point_index, radii, point_indices.indices, distances, 0);

            for (int j = 0; j < point_indices.indices.size(); j++)
            {
              if(random_forest_posterior.at<float>(point_indices.indices[j]) < posterior.at<float>(valid_points))
                 random_forest_posterior.at<float>(point_indices.indices[j]) = posterior.at<float>(valid_points);
            }
            valid_points++;
        }
    }

//----VISUALIZE COLORED RANDOM FOREST POSTERIOR

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_classified_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_classified_cloud_RGB);

//    for (int i = 0; i < ptr_classified_cloud_RGB->size(); i++)
//    {
//        ptr_classified_cloud_RGB->at(i).r = 255;
//        ptr_classified_cloud_RGB->at(i).g = 0;
//        ptr_classified_cloud_RGB->at(i).b = 0;
//    }

//    valid_points = 0;
//    for (int i = 0; i < all_point_indices.size(); i++)
//    {
//        if (mask.at<unsigned char>(i))
//        {
//            pcl::PointIndices point_indices;
//            std::vector<float>  distances;

//            float radii = 0.015;
//            int point_index = all_point_indices[i];

//            int r, g, b;
//        //	std::cout << posterior.at<float>(i) << "   ";

//            r = 255.0f * (1.0 - posterior.at<float>(valid_points));
//            g = 255.0f * posterior.at<float>(valid_points);
//            b = 0.0;

//            ptr_kd_tree_RGB->radiusSearch(*ptr_scene_cloud_RGB, point_index, radii, point_indices.indices, distances, 0);

//            for (int j = 0; j < point_indices.indices.size(); j++)
//            {
//                if(posterior.at<float>(point_indices.indices[j]) < posterior.at<float>(valid_points))
//                {
//                    ptr_classified_cloud_RGB->at(point_indices.indices.at(j)).r = r;
//                    ptr_classified_cloud_RGB->at(point_indices.indices.at(j)).g = g;
//                    ptr_classified_cloud_RGB->at(point_indices.indices.at(j)).b = b;
//                }
//            }
//            valid_points++;
//        }
//    }

//    pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");

//    while (1)
//    {
//        cloud_viewer.showCloud(ptr_bin_cloud_RGB); sleep(1);
//        cloud_viewer.showCloud(ptr_classified_cloud_RGB); sleep(1);
//    }
}


#endif // RANDOM_FOREST_ROUTINES_H
