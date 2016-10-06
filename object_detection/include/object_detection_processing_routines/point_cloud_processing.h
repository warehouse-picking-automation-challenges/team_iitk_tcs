#ifndef POINT_CLOUD_PROCESSING_H
#define POINT_CLOUD_PROCESSING_H

//------CLOUD MANIPULATION

void cloud_to_image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB, cv::Mat& image)
{
    image.release();
    image.create(ptr_scene_cloud_RGB->size(), 1, CV_32FC3);

    for (int i = 0; i < ptr_scene_cloud_RGB->size(); i++)
    {
        float* pixel = (float*)(image.data + i* image.step[0]);
        pixel[0] = (float)ptr_scene_cloud_RGB->at(i).b / 255.0;
        pixel[1] = (float)ptr_scene_cloud_RGB->at(i).g / 255.0;
        pixel[2] = (float)ptr_scene_cloud_RGB->at(i).r / 255.0;
    }
}

void get_rectangle_for_cropping_image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ptr_pixel_cloud_RGB, cv::Rect2i &rect_cropped_image)
{
    int min_x,max_x,min_y,max_y;
    min_x = 10000;
    max_x = -10000;
    min_y = 10000;
    max_y = -10000;

    for( int i=0;i< ptr_pixel_cloud_RGB->size();i++)
    {
     if(ptr_pixel_cloud_RGB->at(i).x < min_x)
         min_x = ptr_pixel_cloud_RGB->at(i).x;

     if(ptr_pixel_cloud_RGB->at(i).x > max_x)
         max_x = ptr_pixel_cloud_RGB->at(i).x;


     if(ptr_pixel_cloud_RGB->at(i).y < min_y)
          min_y = ptr_pixel_cloud_RGB->at(i).y;

      if(ptr_pixel_cloud_RGB->at(i).y > max_y)
         max_y = ptr_pixel_cloud_RGB->at(i).y;
    }
    rect_cropped_image.x = min_x;
    rect_cropped_image.y = min_y;
    rect_cropped_image.width = max_x-  min_x +1;
    rect_cropped_image.height = max_y-  min_y +1;
}

void construct_registred_image_RGB_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB)
{
    ptr_pixel_cloud_RGB->clear();

    for (int i = 0; i < ptr_cloud->height; i++)
        for (int j = 0; j < ptr_cloud->width; j++)
        {
            pcl::PointXYZRGB point;
            point.x = j;
            point.y = i;
            point.z = 0;
            point.r = ptr_cloud->at(j, i).r;
            point.g = ptr_cloud->at(j, i).g;
            point.b = ptr_cloud->at(j, i).b;
            ptr_pixel_cloud_RGB->push_back(point);
        }
}

void update_registered_image_and_mask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud,
    cv::Mat& image_registered_rgb, cv::Mat& image_mask)
{
//    image_mask = 0;
//    image_registered_rgb = 0;
    image_mask = cv::Mat::zeros(image_mask.size(),image_mask.type());

    image_registered_rgb = cv::Mat::zeros(image_registered_rgb.size(),image_registered_rgb.type());

    for (int i = 0; i < ptr_pixel_cloud->size(); i++)
    {
        int row = (int)ptr_pixel_cloud->at(i).y;
        int col = (int)ptr_pixel_cloud->at(i).x;
        unsigned char  r = ptr_pixel_cloud->at(i).r;
        unsigned char  g = ptr_pixel_cloud->at(i).g;
        unsigned char  b = ptr_pixel_cloud->at(i).b;

        //	printf("%f %f %d %d %d\n", ptr_pixel_cloud->at(i).x, ptr_pixel_cloud->at(i).y, row, col, r, g, b);

        (image_registered_rgb.data + row*image_registered_rgb.step[0] + col*image_registered_rgb.step[1])[0] = b;
        (image_registered_rgb.data + row*image_registered_rgb.step[0] + col*image_registered_rgb.step[1])[1] = g;
        (image_registered_rgb.data + row*image_registered_rgb.step[0] + col*image_registered_rgb.step[1])[2] = r;

        *(image_mask.data + row*image_mask.step[0] + col* image_mask.step[1]) = 0xFF;
    }
}

//----COLOR PROBABILITY IMAGES AND CLOUDS

void get_color_probability_image(cv::Mat& mtx_back_projection, cv::Mat& mtx_colored_probability)
{
    double min, max;
    cv::minMaxLoc(mtx_back_projection, &min, &max);

    std::cout << min << "  " << max << std::endl;

    float hue_min = 240.0f;
    float hue_max = 360.0f;

    float hue_resolution = (hue_max - hue_min) / (max - min);

    mtx_colored_probability.release();
    mtx_colored_probability.create(mtx_back_projection.rows, 1, CV_32FC3);

    for (int i = 0; i < mtx_back_projection.rows; i++)
    {
        float probability = mtx_back_projection.at<float>(i);

        float hue = hue_resolution *(probability - min) + hue_min;
        float sat = 1.0f;
        float val = 1.0f;

        //if (probability < 0.4f)val = 0.0f;

        //printf("%f\n", hue);

        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[0] = hue;
        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[1] = sat;
        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[2] = val;
    }

    cv::cvtColor(mtx_colored_probability, mtx_colored_probability, CV_HSV2BGR);
    mtx_colored_probability.convertTo(mtx_colored_probability, CV_8UC3, 255.0f);
}

void get_back_projected_clouds(std::vector<cv::Mat>& vector_back_projections_and_sum,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vector_ptr_backprojected_clouds_RGB)
{
    for (int i = 0; i < vector_back_projections_and_sum.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_back_projection_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int j = 0; j < ptr_bin_cloud_RGB->size(); j++)
        {
            pcl::PointXYZRGB point;
            point.x = ptr_bin_cloud_RGB->at(j).x;
            point.y = ptr_bin_cloud_RGB->at(j).y;
            point.z = ptr_bin_cloud_RGB->at(j).z;
            point.r = 255.0 * vector_back_projections_and_sum[i].at<float>(j);
            point.g = 255.0 * vector_back_projections_and_sum[i].at<float>(j);
            point.b = 255.0 * vector_back_projections_and_sum[i].at<float>(j);

            ptr_back_projection_cloud->push_back(point);
        }
        vector_ptr_backprojected_clouds_RGB.push_back(ptr_back_projection_cloud);
    }
}

void get_color_probability_image_and_cloud(cv::Mat& mtx_back_projection, cv::Mat& mtx_colored_probability,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_colored_probability_cloud_RGB)
{
    double min, max;
    cv::minMaxLoc(mtx_back_projection, &min, &max);

    std::cout << min << "  " << max << std::endl;

    float hue_min = 240.0f;
    float hue_max = 360.0f;

    float hue_resolution = (hue_max - hue_min) / (max - min);

    mtx_colored_probability.release();
    mtx_colored_probability.create(mtx_back_projection.rows, 1, CV_32FC3);

    for (int i = 0; i < mtx_back_projection.rows; i++)
    {
        float probability = mtx_back_projection.at<float>(i);

        float hue = hue_resolution *(probability - min) + hue_min;
        float sat = 1.0f;
        float val = 1.0f;

        //if (probability < 0.4f)val = 0.0f;

        //printf("%f\n", hue);

        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[0] = hue;
        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[1] = sat;
        ((float*)(mtx_colored_probability.data + i*mtx_colored_probability.step[0]))[2] = val;
    }

    cv::cvtColor(mtx_colored_probability, mtx_colored_probability, CV_HSV2BGR);
    mtx_colored_probability.convertTo(mtx_colored_probability, CV_8UC3, 255.0f);

    //imshow("PROBABILITY IMAGE", mtx_colored_probability);
    //cv::waitKey(1);

    for (int i = 0; i < ptr_bin_cloud_RGB->size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = ptr_bin_cloud_RGB->at(i).x;
        point.y = ptr_bin_cloud_RGB->at(i).y;
        point.z = ptr_bin_cloud_RGB->at(i).z;

        unsigned char* pixel = (mtx_colored_probability.data + i*mtx_colored_probability.step[0]);
        point.r = pixel[2];
        point.g = pixel[1];
        point.b = pixel[0];
        ptr_colored_probability_cloud_RGB->push_back(point);
    }

}


//-----CLOUD FILTERING


void remove_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_pixel_cloud, pcl::IndicesConstPtr& ptr_removed_point_indices)
{
    pcl::ExtractIndices<pcl::PointXYZ> indices_extractor(true);

    indices_extractor.setIndices(ptr_removed_point_indices);
    indices_extractor.setNegative(true);
    indices_extractor.setInputCloud(ptr_pixel_cloud);
    indices_extractor.filter(*ptr_pixel_cloud);

    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered_pixel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j = 0;
    for (int i = 0; i < ptr_pixel_cloud->size(); i++)
    {
    if (i != removed_point_indices.indices.at(j))
    {
    ptr_filtered_pixel_cloud->push_back(ptr_pixel_cloud->at(i));
    continue;
    }
    j++;
    }

    ptr_pixel_cloud->clear();
    pcl::copyPointCloud(*ptr_filtered_pixel_cloud, *ptr_pixel_cloud);
    */
}

void remove_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud, pcl::IndicesConstPtr& ptr_removed_point_indices)
{
    pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor(true);

    indices_extractor.setIndices(ptr_removed_point_indices);
    indices_extractor.setNegative(true);
    indices_extractor.setInputCloud(ptr_pixel_cloud);
    indices_extractor.filter(*ptr_pixel_cloud);

    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered_pixel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j = 0;
    for (int i = 0; i < ptr_pixel_cloud->size(); i++)
    {
    if (i != removed_point_indices.indices.at(j))
    {
    ptr_filtered_pixel_cloud->push_back(ptr_pixel_cloud->at(i));
    continue;
    }
    j++;
    }

    ptr_pixel_cloud->clear();
    pcl::copyPointCloud(*ptr_filtered_pixel_cloud, *ptr_pixel_cloud);
    */
}

void filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_pixel_cloud,
    bool filter_pass_through = false, bool filter_statistically = false, bool filter_radially = false,
    unsigned char* mask = NULL, float* field_limits = NULL, float std_dev = 0.9f, int nebrs = 5)
{
    unsigned char local_mask[3];
    if (!mask)
        for (int i = 0; i<3; i++)local_mask[i] = 1;
    else
        for (int i = 0; i<3; i++)local_mask[i] = mask[i];

    pcl::IndicesConstPtr ptr_removed_point_indices;

    if (source_cloud_ptr != filtered_cloud_ptr)
        pcl::copyPointCloud(*source_cloud_ptr, *filtered_cloud_ptr);

    if (filter_pass_through)
    {
        std::string field_name[] = { "x", "y", "z" };
        pcl::PassThrough<pcl::PointXYZ> passthrough_filter(true);

        for (int i = 0; i < 3; i++)
        {
            if (local_mask[i])
            {
                passthrough_filter.setFilterFieldName(field_name[i].c_str());
                passthrough_filter.setFilterLimits(field_limits[i * 2], field_limits[i * 2 + 1]);
                passthrough_filter.setInputCloud(filtered_cloud_ptr);
                passthrough_filter.filter(*filtered_cloud_ptr);


                ptr_removed_point_indices = passthrough_filter.getRemovedIndices();

                std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
                if (ptr_pixel_cloud && ptr_removed_point_indices->size())
                {
                   std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
                    remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
                }
            }
        }
    }

    if (filter_statistically)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_filter(true);
        statistical_filter.setMeanK(40);
        statistical_filter.setStddevMulThresh(std_dev);  //.7 for estimation
        statistical_filter.setInputCloud(filtered_cloud_ptr);
        statistical_filter.filter(*filtered_cloud_ptr);

        ptr_removed_point_indices = statistical_filter.getRemovedIndices();

        std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
        if (ptr_pixel_cloud && ptr_removed_point_indices->size())
        {
           std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
            remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
        }
    }

    if (filter_radially)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radial_filter(true);
        radial_filter.setRadiusSearch(.01);
        radial_filter.setMinNeighborsInRadius(nebrs);
        radial_filter.setInputCloud(filtered_cloud_ptr);
        if (filter_radially)radial_filter.filter(*filtered_cloud_ptr);

        ptr_removed_point_indices = radial_filter.getRemovedIndices();

        std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
        if (ptr_pixel_cloud && ptr_removed_point_indices->size())
        {
           std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
            remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
        }
    }
}

void filter_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud,
    bool filter_pass_through = false, bool filter_statistically = false, bool filter_radially = false,
    unsigned char* mask = NULL, float* field_limits = NULL, float std_dev = 0.9f, int nebrs = 5)
{
    unsigned char local_mask[3];
    if (!mask)
        for (int i = 0; i<3; i++)local_mask[i] = 1;
    else
        for (int i = 0; i<3; i++)local_mask[i] = mask[i];

    pcl::IndicesConstPtr ptr_removed_point_indices;

    if (source_cloud_ptr != filtered_cloud_ptr)
    pcl::copyPointCloud(*source_cloud_ptr, *filtered_cloud_ptr);

    if (filter_pass_through)
    {
        std::string field_name[] = { "x", "y", "z" };
        pcl::PassThrough<pcl::PointXYZRGB> passthrough_filter(true);

        for (int i = 0; i < 3; i++)
        {
            if (local_mask[i])
            {
                passthrough_filter.setFilterFieldName(field_name[i].c_str());
                passthrough_filter.setFilterLimits(field_limits[i * 2], field_limits[i * 2 + 1]);
                passthrough_filter.setInputCloud(filtered_cloud_ptr);
                passthrough_filter.filter(*filtered_cloud_ptr);


                ptr_removed_point_indices = passthrough_filter.getRemovedIndices();

                std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
                if (ptr_pixel_cloud && ptr_removed_point_indices->size())
                {
                   std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
                    remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
                }
            }
        }
    }

    if (filter_statistically)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter(true);
        statistical_filter.setMeanK(40);
        statistical_filter.setStddevMulThresh(std_dev);  //.7 for estimation
        statistical_filter.setInputCloud(filtered_cloud_ptr);
        statistical_filter.filter(*filtered_cloud_ptr);

        ptr_removed_point_indices = statistical_filter.getRemovedIndices();

        std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
        if (ptr_pixel_cloud && ptr_removed_point_indices->size())
        {
           std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
            remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
        }
    }

    if (filter_radially)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radial_filter(true);
        radial_filter.setRadiusSearch(.01);
        radial_filter.setMinNeighborsInRadius(nebrs);
        radial_filter.setInputCloud(filtered_cloud_ptr);
        if (filter_radially)radial_filter.filter(*filtered_cloud_ptr);

        ptr_removed_point_indices = radial_filter.getRemovedIndices();
        std::cout << "SOURCE FILTERED = " << source_cloud_ptr->size() << "  " <<  filtered_cloud_ptr->size() << std::endl;
        if (ptr_pixel_cloud && ptr_removed_point_indices->size())
        {
           std::cout << "removed point indices = " << ptr_removed_point_indices->size() << "  " << ptr_pixel_cloud->size() << std::endl;
            remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
        }
    }
}

#endif // POINT_CLOUD_PROCESSING_H
