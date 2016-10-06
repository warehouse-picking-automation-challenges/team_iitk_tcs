#ifndef FEATURE_COMPUTING_H
#define FEATURE_COMPUTING_H


//----_CONST VALS

const float radius_normal_search = 0.015f;
const float radius_mean_var = 0.02f;
const float radius_principle_curvature = 0.02f;
const float multiplier_principle_curvature =4.0f;
const float max_angle_limit = 100;

const float max_limit_histogram_mean_var = 100.0f;
const float radius_histogram_mean_var = 0.015f;
const float max_limit_histogram_gradient = 100.0f;
const float radius_intensity_gradient = 0.015f;
const float radius_neighbour_intensity_gradient = 0.015f;

//---ENDS


void process_hsv_image_flat_gray_scale_partition(cv::Mat& image)
{
    cv::cvtColor(image, image, CV_BGR2HSV);

    float sat_min = 0.10f;
    float sat_max = 0.26f;
    float val_min = 0.10f;
    float val_max = 0.20f;

    cv::Mat region_image;

    region_image.create(image.rows, image.cols, CV_8UC1);


    cv::Mat splitted[3];
    cv::split(image, splitted);

    int HUE = 0;
    int SAT = 1;
    int VAL = 2;

    bool SHIFTED = false;

    float hue_offset_limit = 10.0;
    float absolute_hue_max = 360.01;

    float no_of_partitions = 1.01;

    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
        {
            //R1 true color
            if (splitted[SAT].at<float>(i, j) >= sat_max && splitted[VAL].at<float>(i, j) >= val_max)
            {
                float hue_max = 360.01f;

                if (SHIFTED)
                {
                    splitted[HUE].at<float>(i, j) += hue_offset_limit;

                    if (splitted[HUE].at<float>(i, j)> absolute_hue_max)
                        splitted[HUE].at<float>(i, j) -= absolute_hue_max;
                }

                splitted[HUE].at<float>(i, j) /= hue_max;
                splitted[SAT].at<float>(i, j) = 0.0f;
                region_image.at<unsigned char>(i, j) = 0xFF;
            } // R31
            else if (splitted[SAT].at<float>(i, j) >= sat_min && splitted[SAT].at<float>(i, j) < sat_max && splitted[VAL].at<float>(i, j) >= val_max)
            {
                splitted[HUE].at<float>(i, j) = splitted[VAL].at<float>(i, j);
                splitted[SAT].at<float>(i, j) = 1.0f / no_of_partitions;
                region_image.at<unsigned char>(i, j) = 0x00;
            }// R32
            else if (splitted[SAT].at<float>(i, j) >= sat_max && splitted[VAL].at<float>(i, j) >= val_min && splitted[VAL].at<float>(i, j) <= val_max)
            {
                splitted[HUE].at<float>(i, j) = splitted[VAL].at<float>(i, j);
                splitted[SAT].at<float>(i, j) = 1.0f / no_of_partitions;
                region_image.at<unsigned char>(i, j) = 0x00;
            }// R3
            else
            {
                splitted[HUE].at<float>(i, j) = splitted[VAL].at<float>(i, j);
                splitted[SAT].at<float>(i, j) = 1.0f / no_of_partitions;
                region_image.at<unsigned char>(i, j) = 0x00;
            }
            //   R33 LEAVE FOR FUTURE
        }

    cv::merge(splitted, 2, image);

    //cv::imshow("REGION IMAGE", region_image);
    //cv::waitKey(10);
}


//---FEATURES FOR BACK PROJECTION

void compute_depth_mean_var(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_mean_var, cv::Mat& mask)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr ptr_scene_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

    normal_estimator.setRadiusSearch(radius_normal_search);
    normal_estimator.setInputCloud(ptr_scene_cloud);
    normal_estimator.compute(*ptr_scene_normal_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    ptr_kd_tree->setInputCloud(ptr_scene_cloud);

    mask.release();
    mask.create(ptr_scene_normal_cloud->size(), 1, CV_8UC1);

    mask = 0xFF;
    mtx_mean_var.release();
    mtx_mean_var.create(ptr_scene_normal_cloud->size(), 1, CV_32FC2);

    for (int i = 0; i < ptr_scene_normal_cloud->size(); i++)
    {

        pcl::PointIndices point_indices;
        float radii = radius_mean_var;
        std::vector<float>  distances;

        ptr_kd_tree->radiusSearch(*ptr_scene_cloud, i, radii, point_indices.indices, distances, 0);

        std::vector<float> angles;
        float mean = 0.0;

        for (int j = 1; j < point_indices.indices.size(); j++)
        {

            Eigen::Vector3f normal_vec1;
            Eigen::Vector3f normal_vec2;

            normal_vec1.x() = ptr_scene_normal_cloud->at(i).normal_x;
            normal_vec1.y() = ptr_scene_normal_cloud->at(i).normal_y;
            normal_vec1.z() = ptr_scene_normal_cloud->at(i).normal_z;

            normal_vec2.x() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_x;
            normal_vec2.y() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_y;
            normal_vec2.z() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_z;

            float dot_product = normal_vec1.dot(normal_vec2);

            if (isnan(dot_product) || dot_product >1.0)
                continue;

            dot_product -= 0.000000000005;

            float angle = 180.0f *   std::acos(dot_product) / 3.141f;

            angles.push_back(angle);

            mean += angle;
            /*	printf("%f %f %f %f %f %f %f %f %f\n", ptr_scene_normal_cloud->at(i).x,
            ptr_scene_normal_cloud->at(i).y,
            ptr_scene_normal_cloud->at(i).z,
            ptr_scene_normal_cloud->at(i).normal_x,
            ptr_scene_normal_cloud->at(i).normal_y,
            ptr_scene_normal_cloud->at(i).normal_z,
            dot_product, angle, distances.at(j)
            );*/

        }

        mean /= angles.size();

        float var = 0.0;

        for (int j = 0; j < angles.size(); j++)
            var += (angles.at(j) - mean) * (angles.at(j) - mean);

        var /= angles.size();
        var = sqrtf(var);

        //printf("BEFORE Mean =  %f  var = %f \n", mean, var);

        std::vector<float> filtered_angles;

        float outlier_rate = 1.3f;

        for (int j = 0; j < angles.size(); j++)
        {
            if (abs(((angles.at(j) - mean) / var)) <= outlier_rate)
            {
                filtered_angles.push_back(angles.at(j));
            }
        }

        mean = 0.0f;

        for (int j = 0; j < filtered_angles.size(); j++)
            mean += filtered_angles.at(j);

        mean /= filtered_angles.size();

        var = 0.0f;

        for (int j = 0; j < filtered_angles.size(); j++)
            var += (filtered_angles.at(j) - mean) * (filtered_angles.at(j) - mean);

        var /= filtered_angles.size();
        var = sqrtf(var);

        //printf("AFTER Mean =  %f  var = %f \n", mean, var);

        float max_limit = max_angle_limit;

        int invalid = 0;

        if (filtered_angles.size())
        {
            if (isnan(mean) || isnan(var) || mean < 0 || var <0 ||  mean >= max_limit || var >= max_limit)
            {
                //		printf("HI \n");
                //	printf("AFTER Mean =  %f  var = %f \n", mean, var);
                mask.at<unsigned char>(i) = 0;
                invalid++;
                continue;
            }

            mean /= max_limit;
            var /= max_limit;

            ((float*)(mtx_mean_var.data + i* mtx_mean_var.step[0]))[0] = mean;
            ((float*)(mtx_mean_var.data + i* mtx_mean_var.step[0]))[1] = var;

            // printf("%f %f \n ", mean, var);
        }
        else
            mask.at<unsigned char>(i) = 0;


    }
}

void compute_depth_principle_curvature(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_principle_curvature, cv::Mat& mask)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr ptr_scene_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

    normal_estimator.setRadiusSearch(radius_normal_search);
    normal_estimator.setInputCloud(ptr_scene_cloud);
    normal_estimator.compute(*ptr_scene_normal_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr ptr_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    ptr_kd_tree->setInputCloud(ptr_scene_cloud);

    mask.release();
    mask.create(ptr_scene_normal_cloud->size(), 1, CV_8UC1);

    mask = 0xFF;
    mtx_principle_curvature.release();
    mtx_principle_curvature.create(ptr_scene_normal_cloud->size(), 1, CV_32FC2);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PrincipalCurvatures> principle_curvature_estimation;

    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr ptr_principle_curvature_cloud(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    principle_curvature_estimation.setInputNormals(ptr_scene_normal_cloud);
    principle_curvature_estimation.setRadiusSearch(radius_principle_curvature);
    principle_curvature_estimation.setInputCloud(ptr_scene_cloud);
    principle_curvature_estimation.compute(*ptr_principle_curvature_cloud);

    for (int i = 0; i < ptr_principle_curvature_cloud->size(); i++)
    {
        float k1 = ptr_principle_curvature_cloud->at(i).pc1;
        float k2 = ptr_principle_curvature_cloud->at(i).pc2;

        float a = k1*k2;
        float b = (k1 + k2) / 2;

        k1 *= multiplier_principle_curvature;
        k2 *= multiplier_principle_curvature;

        //printf("%f %f %f %f\n", k1, k2, a, b);

        if (k1< 0 || k2 < 0 || k1 >= 1.0f || k2 >= 1.0f || isnan(k1) || isnan(k2))
        {
            mask.at<unsigned char>(i) = 0;
            continue;
        }

        //if(k2!=0.0000f)k1 = k1/k2;

        //k2 = 0;
        if (k1 != 0.0 && k2 != 0.0)
        {
            //k1 /= (k1 + k2);
            //k2 /= (k1 + k2);
        }

        ((float*)(mtx_principle_curvature.data + i*mtx_principle_curvature.step[0]))[0] = k1;
        ((float*)(mtx_principle_curvature.data + i*mtx_principle_curvature.step[0]))[1] = k2;
    }
}

void compute_cloud_to_color_image_processed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud, cv::Mat& color_image)
{
    color_image.release();
    color_image.create(ptr_scene_cloud->size(), 1, CV_8UC3);

    for (int i = 0; i < ptr_scene_cloud->size(); i++)
    {
        (color_image.data + i*color_image.step[0])[0] = ptr_scene_cloud->at(i).b;
        (color_image.data + i*color_image.step[0])[1] = ptr_scene_cloud->at(i).g;
        (color_image.data + i*color_image.step[0])[2] = ptr_scene_cloud->at(i).r;
    }

    color_image.convertTo(color_image, CV_32FC3, 1.0f / 255.0f);
    process_hsv_image_flat_gray_scale_partition(color_image);
}


//---FEATURE COMPUTING FOR MODEL LEARNING

void compute_histogram_gaussian_partitioned(std::vector<float>& angles, std::vector<float>& weights, int row, cv::Mat& mtx_histogram,
                                            float resolution, float var, float max_limit)
{
    int hist_dimension = mtx_histogram.cols;

    for (int i = 0; i < angles.size(); i++)
    {
        int angle_weights;
        if (weights.size())
            angle_weights = weights[i];
        else
            angle_weights = 1.0f;

        for (int k = 0; k < mtx_histogram.cols; k++)
        {
            float mean = k * max_limit / ((float)hist_dimension - 1.0f);
            float angle = angles[i];

            float weight = (angle - mean) / var;
            weight = -((weight*weight) / 2.0f);
            weight = std::exp(weight);
            mtx_histogram.at<float>(row, k) += (angle_weights*weight);
        }
    }
    cv::normalize(mtx_histogram.row(row), mtx_histogram.row(row), 0.0f, 1.0f, cv::NORM_MINMAX);
    //	std::cout << " MEAN_VAR HISTOGRAM" << std::endl << mtx_mean_var_histogram.row(row) << std::endl;
}


void compute_depth_mean_var_histogram_feature_gaussian_partition(pcl::search::KdTree<pcl::PointXYZ>::Ptr& ptr_kd_tree,
                                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
                                                                 pcl::PointCloud<pcl::PointNormal>::Ptr& ptr_scene_normal_cloud,
                                                                 cv::Mat& mtx_mean_var_histogram, cv::Mat& mask,
                                                                 std::vector<int>& all_point_indices)
{
    mask.release();
    mask.create(all_point_indices.size(), 1, CV_8UC1);
    mask = 0xFF;

    float min_limit = 0.0f;
    float  max_limit = max_limit_histogram_mean_var;

    float resolution = 10.0f;  //degrees
    int hist_dimension = (int)((max_limit - min_limit) / resolution) + 1;

    resolution /= (max_limit - min_limit);
    float fuzzy_var = 0.20 * resolution;


    mtx_mean_var_histogram.release();
    mtx_mean_var_histogram.create(all_point_indices.size(), hist_dimension, CV_32FC1);
    mtx_mean_var_histogram = 0.0f;

    for (int i = 0; i < all_point_indices.size(); i++)
    {
        pcl::PointIndices point_indices;
        std::vector<float>  distances;

        float radii = radius_histogram_mean_var;
        int point_index = all_point_indices[i];

        ptr_kd_tree->radiusSearch(*ptr_scene_cloud, point_index, radii, point_indices.indices, distances, 0);

        std::vector<float> angles;
        float mean = 0.0f;

        for (int j = 0; j < point_indices.indices.size(); j++)
        {

            Eigen::Vector3f normal_vec1;
            Eigen::Vector3f normal_vec2;

            normal_vec1.x() = ptr_scene_normal_cloud->at(point_index).normal_x;
            normal_vec1.y() = ptr_scene_normal_cloud->at(point_index).normal_y;
            normal_vec1.z() = ptr_scene_normal_cloud->at(point_index).normal_z;

            normal_vec2.x() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_x;
            normal_vec2.y() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_y;
            normal_vec2.z() = ptr_scene_normal_cloud->at(point_indices.indices.at(j)).normal_z;

            float dot_product = normal_vec1.dot(normal_vec2);

            if (isnan(dot_product) || dot_product > 1.0)
                continue;

            dot_product -= 0.000000000005f;

            float angle = 180.0f * std::acos(dot_product) / 3.141f;

            angles.push_back(angle);
            mean += angle;

            //printf("%f %f %f %f %f %f %f %f %f\n", ptr_scene_normal_cloud->at(i).x, ptr_scene_normal_cloud->at(i).y,ptr_scene_normal_cloud->at(i).z,
            //ptr_scene_normal_cloud->at(i).normal_x,	ptr_scene_normal_cloud->at(i).normal_y,	ptr_scene_normal_cloud->at(i).normal_z, dot_product, angle, distances.at(j));
        }


        mean /= angles.size();

        float var = 0.0;
        for (int j = 0; j < angles.size(); j++)
            var += (angles.at(j) - mean) * (angles.at(j) - mean);

        var /= angles.size();
        var = sqrtf(var);

        std::vector<float> filtered_angles;

        float outlier_rate = 1.2f;
        for (int j = 0; j < angles.size(); j++)
        {
            if (abs(((angles.at(j) - mean) / var)) <= outlier_rate)
            {
                filtered_angles.push_back(angles.at(j));
            }
        }


        angles.clear();
        if (filtered_angles.size())
        {
            for (int j = 0; j < filtered_angles.size(); j++)
            {
                if (filtered_angles[j] >= min_limit && filtered_angles[j] < max_limit)
                {
                    filtered_angles[j] -= min_limit;
                    filtered_angles[j] /= (max_limit - min_limit);
                    angles.push_back(filtered_angles[j]);
                }
            }
        }

        if (angles.size())
        {
            std::vector<float> vector_temp;
            compute_histogram_gaussian_partitioned(angles, vector_temp, i, mtx_mean_var_histogram, resolution, fuzzy_var, 1.0);
            //		std::cout << " MEAN_VAR HISTOGRAM" << std::endl << mtx_mean_var_histogram.row(i) << std::endl;
            //	printf("%f %f \n ", mean, var);
        }
        else
            mask.at<unsigned char>(i) = 0;
    }
}

void compute_histogram_gaussian_gray_scale_flat_partition(cv::Mat& image, int row, cv::Mat& color_hist)
{
    const float resolution = 30.0f;  //degrees
    const float var = 0.50 * resolution;

    int hist_size[] = { (int)(360.0f / resolution), 2 };
    int dims = 2;

    cv::Mat hist;
    hist.release();
    hist.create(dims, hist_size, CV_32FC1);
    hist = 0.0f;

    float black_max = 0.20f;
    float gray_min = 0.30f;
    float gray_max = 0.70f;
    float white_min = 0.80f;


    for (int i = 0; i < image.rows; i++)
    {
        //if (mask.at<unsigned char>(i, j))
        {
            float* pixel = ((float*)(image.data + i* image.step[0]));
            int sat = hist_size[1] * pixel[1];

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
                    hist.at<float>(k, sat) += weight;
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
                    hist.at<float>(k, sat) += weight;
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
                    hist.at<float>(k, sat) += weight;
                }
            }
        }
    }


    cv::normalize(hist.col(0), hist.col(0), 0.0f, 1.0f, cv::NORM_MINMAX);
    cv::normalize(hist.col(1), hist.col(1), 0.0f, 1.0f, cv::NORM_MINMAX);

    for (int i = 0; i < hist.rows; i++)
        color_hist.at<float>(row, i) = hist.at<float>(i, 0);
    for (int i = 0; i < 3; i++)
        color_hist.at<float>(row, i + hist.rows) = hist.at<float>(i, 1);

    //std::cout << hist << std::endl;
    //std::cout << color_hist.row(row) << std::endl;

    //for (int i = 0; i < hist_size[0]; i++)
    //std::cout << hist.at<float>(i, 0) << " ," << hist.at<float>(i, 1) << " ," << hist.at<float>(i, 2) << std::endl;

}

void compute_color_histogram_feature(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr& ptr_kd_tree_RGB,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
                                     cv::Mat& mtx_color_histogram, std::vector<int>& all_point_indices)
{
    float resolution = 30.0f;
    int hist_dimension = (int)(360.0f / resolution) + 3;

    mtx_color_histogram.release();
    mtx_color_histogram.create(all_point_indices.size(), hist_dimension, CV_32FC1);
    mtx_color_histogram = 0;


    for (int i = 0; i < all_point_indices.size(); i++)
    {
        pcl::PointIndices point_indices;
        std::vector<float>  distances;

        float radii = 0.015;
        int point_index = all_point_indices[i];

        ptr_kd_tree_RGB->radiusSearch(*ptr_scene_cloud_RGB, point_index, radii, point_indices.indices, distances, 0);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_voxel_point_neighbourhood_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int j = 0; j < point_indices.indices.size(); j++)
            ptr_voxel_point_neighbourhood_scene_cloud_RGB->push_back(ptr_scene_cloud_RGB->at(point_indices.indices[j]));

        cv::Mat color_image;
        cloud_to_image(ptr_voxel_point_neighbourhood_scene_cloud_RGB, color_image);
        process_hsv_image_flat_gray_scale_partition(color_image);
        compute_histogram_gaussian_gray_scale_flat_partition(color_image, i, mtx_color_histogram);
    }
}

void compute_gradient_histogram_feature(pcl::search::KdTree<pcl::PointXYZI>::Ptr& ptr_kd_tree_I,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr& ptr_scene_cloud_I,
                                        pcl::PointCloud<pcl::PointNormal>::Ptr& ptr_scene_normal_cloud,
                                        cv::Mat& mtx_gradient_histogram, cv::Mat& mask, std::vector<int>& all_point_indices)
{

    mask.release();
    mask.create(all_point_indices.size(), 1, CV_8UC1);
    mask = 0xFF;

    float min_limit = 0.0f;
    float  max_limit = max_limit_histogram_gradient;

    float resolution = 10.0f;  //degrees
    int hist_dimension = (int)((max_limit - min_limit) / resolution) + 1;

    resolution /= (max_limit - min_limit);
    float fuzzy_var = 0.20 * resolution;

    mtx_gradient_histogram.release();
    mtx_gradient_histogram.create(all_point_indices.size(), hist_dimension, CV_32FC1);
    mtx_gradient_histogram = 0.0f;


    pcl::PointCloud<pcl::IntensityGradient>::Ptr ptr_intensity_gradient_cloud(new pcl::PointCloud<pcl::IntensityGradient>);

    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::IntensityGradient> intensity_gradient_estimation;
    intensity_gradient_estimation.setSearchMethod(ptr_kd_tree_I);
    intensity_gradient_estimation.setInputCloud(ptr_scene_cloud_I);
    intensity_gradient_estimation.setRadiusSearch(radius_intensity_gradient);
    intensity_gradient_estimation.setInputNormals(ptr_scene_normal_cloud);

    intensity_gradient_estimation.compute(*ptr_intensity_gradient_cloud);

    for (int i = 0; i < all_point_indices.size(); i++)
    {
        pcl::PointIndices point_indices;
        std::vector<float>  distances;

        float radii = radius_neighbour_intensity_gradient;
        int point_index = all_point_indices[i];

        ptr_kd_tree_I->radiusSearch(*ptr_scene_cloud_I, point_index, radii, point_indices.indices, distances, 0);

        std::vector<float> angles;
        std::vector<float> magnitude_weights;
        float mean = 0.0f;

        for (int j = 0; j < point_indices.indices.size(); j++)
        {

            Eigen::Vector3f gradient_vec1;
            Eigen::Vector3f gradient_vec2;

            gradient_vec1.x() = ptr_intensity_gradient_cloud->at(point_index).gradient_x;
            gradient_vec1.y() = ptr_intensity_gradient_cloud->at(point_index).gradient_y;
            gradient_vec1.z() = ptr_intensity_gradient_cloud->at(point_index).gradient_z;

            gradient_vec2.x() = ptr_intensity_gradient_cloud->at(point_indices.indices.at(j)).gradient_x;
            gradient_vec2.y() = ptr_intensity_gradient_cloud->at(point_indices.indices.at(j)).gradient_y;
            gradient_vec2.z() = ptr_intensity_gradient_cloud->at(point_indices.indices.at(j)).gradient_z;


            float dot_product = gradient_vec1.dot(gradient_vec2);

            if (isnan(dot_product))
                continue;

            float magnitude_weight = gradient_vec1.norm()* gradient_vec2.norm();
            magnitude_weights.push_back(magnitude_weight);

            gradient_vec1.normalize();
            gradient_vec2.normalize();
            dot_product = gradient_vec1.dot(gradient_vec2);

            if(isnan(dot_product) || dot_product > 1.0 )
                continue;
            dot_product -= 0.000005f;

            float angle = 180.0f * std::acos(dot_product) / 3.141f;

            angle = std::abs(angle - 360.0f * std::abs(((int)(angle / 360.0f))));

            //printf("%f %f %f  \n", angle,magnitude_weight,dot_product);

            angles.push_back(angle);
            mean += angle;

            //printf("%f %f %f %f %f %f %f %f %f\n", ptr_scene_normal_cloud->at(i).x, ptr_scene_normal_cloud->at(i).y,ptr_scene_normal_cloud->at(i).z,
            //ptr_scene_normal_cloud->at(i).normal_x,	ptr_scene_normal_cloud->at(i).normal_y,	ptr_scene_normal_cloud->at(i).normal_z, dot_product, angle, distances.at(j));

        }

        mean /= angles.size();

        float var = 0.0;
        for (int j = 0; j < angles.size(); j++)
            var += (angles.at(j) - mean) * (angles.at(j) - mean);

        var /= angles.size();
        var = sqrtf(var);

        //printf("BEFORE Mean =  %f  var = %f \n", mean, var);

        std::vector<float> filtered_angles;
        std::vector<float> filtered_magnitude_weights;

        float outlier_rate = 1.2f;

        for (int j = 0; j < angles.size(); j++)
        {
            if (abs(((angles.at(j) - mean) / var)) <= outlier_rate)
            {
                filtered_angles.push_back(angles.at(j));
                filtered_magnitude_weights.push_back(magnitude_weights[j]);
            }
        }

        angles.clear();
        magnitude_weights.clear();

        if (filtered_angles.size())
        {
            for (int j = 0; j < filtered_angles.size(); j++)
            {
                if (filtered_angles[j] >= min_limit && filtered_angles[j] < max_limit)
                {
                    //printf("%f %f \n", filtered_angles[j],filtered_magnitude_weights[j]);
                    filtered_angles[j] -= min_limit;
                    filtered_angles[j] /= (max_limit - min_limit);
                    angles.push_back(filtered_angles[j]);
                    magnitude_weights.push_back(filtered_magnitude_weights[j]);
                }
            }
        }
        //	printf("\n");
        if (angles.size())
        {
            compute_histogram_gaussian_partitioned(angles, magnitude_weights, i, mtx_gradient_histogram, resolution, fuzzy_var, 1.0);
            //	std::cout << " GRADIENT HISTOGRAM" << std::endl << mtx_gradient_histogram.row(i) << std::endl;
            //	printf("%f %f \n ", mean, var);
        }
        else
            mask.at<unsigned char>(i) = 0;
    }
}



#endif // FEATURE_COMPUTING_H
