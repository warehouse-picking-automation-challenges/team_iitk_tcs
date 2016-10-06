#ifndef BACK_PROJECTION_ROUTINES_H
#define BACK_PROJECTION_ROUTINES_H


void back_project_depth_histogram(cv::Mat& mtx_mean_var, cv::Mat& hist, cv::Mat& mask, cv::Mat& back_projection)
{
    int *hist_size;

    int dims = hist.dims;

    hist_size = new int[dims];

    for (int i = dims - 1; i > 0; i--)
    {
        hist_size[i] = hist.step[i - 1] / hist.step[i];
    }
    hist_size[0] = (hist.dataend - hist.datastart + 1) / hist.step[0];

    std::cout << "HIST SIZE =";
    for (int i = 0; i < dims; i++)std::cout << hist_size[i] << " ";
    std::cout << std::endl;


    back_projection.release();
    back_projection.create(mtx_mean_var.rows, mtx_mean_var.cols, CV_32FC1);

    /*
    for (int j = 0; j < hist_x; j++)
    for (int k = 0; k < hist_y; k++)
    for (int l = 0; l < hist_z; l++)
    {
    std::cout <<  histograms[i].at<float>(j, k, l) << "  ";
    }

    Sleep(1000);
    */

  //  std::cout << " BACK PROJECTION MATRIX CREATED" << std::endl;
    for (int j = 0; j < mtx_mean_var.rows; j++)
    {
        for (int k = 0; k < mtx_mean_var.cols; k++)
        {
            if (mask.at<unsigned char>(j, k))
            {
                int bin_location_x;
                int bin_location_y;

                float* pixel_pointer = (float*)(mtx_mean_var.data + j*mtx_mean_var.step[0] + k*mtx_mean_var.step[1]);

                //std::cout << pixel_pointer[0] << " " << pixel_pointer[1] << " "  ;

                bin_location_x = pixel_pointer[0] * (float)hist_size[0];
                bin_location_y = pixel_pointer[1] * (float)hist_size[1];

                //std::cout << bin_location_x << " " << bin_location_y << " " <<  std::endl;
                //		std::cout << histograms[i].at<float>(bin_location_x, bin_location_y, bin_location_z) << std::endl;
                back_projection.at<float>(j, k) = hist.at<float>(bin_location_x, bin_location_y);
                //		Sleep(10000);
            }
            else
                back_projection.at<float>(j, k) = 0.0f;
        }
    }
}

void back_project_depth_histogram_dense(pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
     cv::Mat& back_projection_sparse, cv::Mat& back_projection_dense)
{
    back_projection_dense.release();
    back_projection_dense.create(ptr_bin_cloud->size(), 1, CV_32FC1);

    int knn = 1;

    for (int i = 0; i < ptr_bin_cloud->size(); i++)
    {
        std::vector<int> indexs(knn);
        std::vector<float> distances(knn);
        kd_tree.nearestKSearch(*ptr_bin_cloud, i, 1, indexs, distances);

        back_projection_dense.at<float>(i) = back_projection_sparse.at<float>(indexs.at(0));
    }
}

void back_project_normals_gaussian_distribution(cv::Mat& hist, cv::Mat& mask,
    cv::Mat& mtx_mean_var,
    pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
    cv::Mat& back_projection_normals_gausssian_distribution)
{
    cv::Mat back_projection_sparse;

    back_project_depth_histogram(mtx_mean_var, hist, mask, back_projection_sparse);
    back_project_depth_histogram_dense(kd_tree, ptr_bin_cloud,back_projection_sparse, back_projection_normals_gausssian_distribution);
}

void back_project_principle_curvatures(cv::Mat& hist, cv::Mat& mask,
    cv::Mat&  mtx_principle_curvature,
    pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
    cv::Mat& back_projection_curvature)
{
    cv::Mat back_projection_sparse;

    back_project_depth_histogram(mtx_principle_curvature, hist, mask, back_projection_sparse);
    back_project_depth_histogram_dense(kd_tree, ptr_bin_cloud, back_projection_sparse, back_projection_curvature);
}

void back_project_partitioned_histogram_fuzzy_gaussian_gray_scale_flat_partition(cv::Mat& processed_image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, cv::Mat& hist, std::vector<cv::Mat>& vector_backprojections)
{

    std::vector<cv::Mat> backprojections(2);

    for (int i = 0; i < backprojections.size(); i++)
    {
        backprojections[i].create(processed_image.rows, processed_image.cols, CV_32FC1);
        backprojections[i] = 0;
    }

    int *hist_size;

    int dims = hist.dims;
    hist_size = new int[dims];

    for (int i = dims - 1; i > 0; i--)
        hist_size[i] = hist.step[i - 1] / hist.step[i];

    hist_size[0] = (hist.dataend - hist.datastart + 1) / hist.step[0];

    std::cout << "HIST SIZE =";
    for (int i = 0; i < dims; i++)std::cout << hist_size[i] << " ";
    std::cout << std::endl;

    float resolution = 30.0f;
    const float var = 0.19 * resolution;

    float black_max = 0.20f;  // FUZZY GRAY SHADES
    float gray_min = 0.30f;
    float gray_max = 0.70f;
    float white_min = 0.80f;


    for (int i = 0; i < processed_image.rows; i++)
        for (int j = 0; j < processed_image.cols; j++)
        {
            float* pixel = ((float*)(processed_image.data + i* processed_image.step[0] + j* processed_image.step[1]));
            int sat = hist_size[1] * pixel[1];
            float back_projection = 0.0f;

            if (sat == 0)
            {
                float hue = pixel[0] * 360.0f;
                for (int k = 0; k < hist_size[0] / 2; k++)
                {
                    float mean = k * 360.0f / (float)hist_size[0];
                    float angle = hue;

                    if (angle >= (mean + 180.0f))
                        angle -= 360.0f;
                    float weight = (angle - mean) / var;
                    weight = -((weight*weight) / 2.0f);
                    weight = std::exp(weight);
                    back_projection += weight * hist.at<float>(k, sat);
                }

                for (int k = hist_size[0] / 2; k < hist_size[0]; k++)
                {
                    float mean = k * 360.0f / (float)hist_size[0];
                    float angle = hue;

                    if (angle <= (float)(((int)mean + (int)180.0f) % (int)360.0f))
                        angle += 360.0f;
                    float weight = (angle - mean) / var;
                    weight = -((weight*weight) / 2.0f);
                    weight = std::exp(weight);
                    back_projection += weight * hist.at<float>(k, sat);
                }

            }
            else
            {
                float hue = pixel[0];
                for (int k = 0; k < 3; k++)
                {
                    float angle = hue;
                    float weight = 0.0;

                    if (k == 0)
                    {
                        if (angle <= black_max)
                            weight = 1.0f;
                        else if (angle > black_max && angle < gray_min)
                        {
                            float x = angle;
                            float x1 = black_max;
                            float y1 = 1.0f;
                            float x2 = gray_min;
                            float y2 = 0.0f;

                            weight = (x - x1)*((y2 - y1) / (x2 - x1)) + y1;
                        }
                    }
                    else if (k == 1)
                    {
                        if (angle >= gray_min && angle <= gray_max)
                            weight = 1.0;
                        else if (angle > black_max && angle < gray_min)
                        {
                            float x = angle;
                            float x1 = black_max;
                            float y1 = 0.0f;
                            float x2 = gray_min;
                            float y2 = 1.0f;

                            weight = (x - x1)*((y2 - y1) / (x2 - x1)) + y1;
                        }
                        else if (angle > gray_max && angle < white_min)
                        {
                            float x = angle;
                            float x1 = gray_max;
                            float y1 = 1.0f;
                            float x2 = white_min;
                            float y2 = 0.0f;

                            weight = (x - x1)*((y2 - y1) / (x2 - x1)) + y1;
                        }
                    }
                    else
                    {
                        if (angle >= white_min)
                            weight = 1.0f;
                        else if (angle > gray_max && angle < white_min)
                        {
                            float x = angle;
                            float x1 = gray_max;
                            float y1 = 0.0f;
                            float x2 = white_min;
                            float y2 = 1.0f;

                            weight = (x - x1)*((y2 - y1) / (x2 - x1)) + y1;
                        }

                    }
                    back_projection += weight * hist.at<float>(k, sat);
                }
            }


            backprojections[sat].at<float>(i, j) = back_projection;
        }

    for (int i = 0; i < backprojections.size(); i++)
        vector_backprojections.push_back(backprojections[i]);
}

void get_all_back_projections(std::vector<int>& bin_members,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
    std::vector<cv::Mat>& vector_back_projections,std::string database_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ptr_bin_cloud_RGB, *ptr_bin_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setDownsampleAllData(true);
    voxel_filter.setLeafSize(0.004, 0.004, 0.004);
    voxel_filter.setInputCloud(ptr_bin_cloud);
    voxel_filter.filter(*ptr_bin_voxel_cloud);

    pcl::search::KdTree<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(ptr_bin_voxel_cloud);


//        pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");
//        while(1)
//        {
//            cloud_viewer.showCloud(ptr_bin_voxel_cloud);
//            sleep(1);
//        }

    std::vector<cv::Mat> vector_masks;
    std::vector<cv::Mat> vector_features;

    int feature_no = 0;
    int mask_no = 0;

    vector_masks.push_back(cv::Mat());
    vector_features.push_back(cv::Mat());
    compute_depth_mean_var(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);

    std::cout << "MEAN VAR FEATURE DONE" <<  std::endl;
    vector_masks.push_back(cv::Mat());
    vector_features.push_back(cv::Mat());
    compute_depth_principle_curvature(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);
    std::cout << "PRINCIPLE CURVATURE FEATURE DONE" << std::endl;

    vector_masks.push_back(cv::Mat());
    vector_features.push_back(cv::Mat());
    compute_cloud_to_color_image_processed(ptr_bin_cloud_RGB, vector_features[feature_no++]);
    std::cout << "COLOR FEATURE DONE" << std::endl;

    //----BIN MEMEBRS[0] CORRESPONDS TO TARGET OBJECT
    std::vector<cv::Mat> vector_histograms;
    read_all_hostograms(bin_members[0], vector_histograms,database_path);

    int back_projection_no = 0;

    vector_back_projections.push_back(cv::Mat());
    back_project_normals_gaussian_distribution(vector_histograms[back_projection_no], vector_masks[back_projection_no],
        vector_features[back_projection_no], kd_tree, ptr_bin_cloud, vector_back_projections[back_projection_no]);
    back_projection_no++;

    std::cout << "DEPTH MEAN VAR BACK PROJECTION DONE" << std::endl;

    vector_back_projections.push_back(cv::Mat());
    back_project_principle_curvatures(vector_histograms[back_projection_no], vector_masks[back_projection_no],
        vector_features[back_projection_no], kd_tree, ptr_bin_cloud, vector_back_projections[back_projection_no]);
    back_projection_no++;

    std::cout << "DEPTH PRINCIPLE CURVATURE DONE" << std::endl;

    back_project_partitioned_histogram_fuzzy_gaussian_gray_scale_flat_partition(vector_features[back_projection_no],
        ptr_bin_cloud_RGB, vector_histograms[back_projection_no], vector_back_projections);

    std::cout << "CURVATURE BACK PROJECTION DONE" << std::endl;

}


#endif // BACK_PROJECTION_ROUTINES_H
