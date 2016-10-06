#ifndef __SHELF_DIMENSIONS_H__
#define __SHELF_DIMENSIONS_H__

//#include<pcl/point_types.h>
//#include<pcl/point_cloud.h>

typedef struct _SHELF_DETAILS
{
	float bin_widths[3];
	float bin_heights[2];

	float bin_inner_heights_from_top[3];

	float rack_inner_width_height[2];
	float rack_outer_width_height[2];

	float  model_origin_offset_from_scene[2];
	float bin_widths_heights[12*2];

	float bin_depth;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_centroid_cloud;

} SHELF_DETAILS;

//------BIN DIMENSIONS

const float SIDE_WALL_REMOVAL_ERROR = 0.009f;
const float BASE_WALL_REMOVAL_ERROR = 0.003f;
const float DEPTH_WALL_REMOVAL_ERROR = 0.01;

const float BIN_CUTTING_CENTROID_ERROR = 0.08;
const float BIN_CUTTING_DEPTH_ERROR = 0.015f;

const float CENTROID_ERROR = 0.06;
const float HEIGHT_ERROR = 0.01;
const float DEPTH_ERROR = 0.01;

const float COORDINATE_OFFSET = 0.01;
const float BASE_PLANE_REMOVAL_THRESHOLD = 0.005f;

//---BIN DIMENSIONS ENDS

//   MODEL BIN_POINTS BASED ON DiMENSION OF THE RACK
void get_shelf_details(SHELF_DETAILS& shelf_details)
{
    float bin_widths[3] = { 0.27f, 0.31f, 0.27f };
    float bin_heights[2] = { 0.23f, 0.19f };

    float bin_inner_heights_from_top[3] = { 0.46f, 0.685f, 0.94f };

    float rack_inner_width_height[2] = { 0.805f, 0.94f };
    float rack_outer_width_height[2] = { 0.850f, 1.00f };

    float  model_origin_offset_from_scene[2];
    float bin_widths_heights[12 * 2];

    float bin_depth = 0.41f;

    float bins_X[3];
    float bins_Y[4];

    bins_X[0] = 0.0f;
    bins_Y[0] = 0.0f;

    bins_X[1] = bin_widths[0] / 2.0f + bin_widths[1] / 2.0f;
    bins_X[2] = bin_widths[0] / 2.0f + bin_widths[1] + bin_widths[2] / 2.0f;

    bins_Y[1] = bin_inner_heights_from_top[0] - bin_heights[0] / 2.0f - bin_heights[1] / 2.0f;
    bins_Y[2] = bin_inner_heights_from_top[1] - bin_heights[0] / 2.0f - bin_heights[1] / 2.0f;
    bins_Y[3] = bin_inner_heights_from_top[2] - bin_heights[0] / 2.0f - bin_heights[0] / 2.0f;

    bins_Y[1] *= -1.0f;
    bins_Y[2] *= -1.0f;
    bins_Y[3] *= -1.0f;



    int offset = 0;
    for (int i = 0; i < 12; i++)
    {
        bin_widths_heights[i * 2] = bin_widths[i % 3];
        if (i < 3 || i > 8)
            bin_widths_heights[i * 2 + 1] = bin_heights[0];
        else
            bin_widths_heights[i * 2 + 1] = bin_heights[1];
    }

    for (int i = 0; i < 12; i++)
        std::cout << bin_widths_heights[i * 2] << "  " << bin_widths_heights[i * 2 + 1] << std::endl;

    const int RACK_DIMS_X = 3;
    const int RACK_DIMS_Y = 4;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < RACK_DIMS_Y; i++)
        for (int j = 0; j < RACK_DIMS_X; j++)
        {
            pcl::PointXYZ point;
            point.x = bins_X[j];
            point.y = bins_Y[i];
            point.z = 0.0f;

            ptr_centroid_cloud->push_back(point);
        }

    float extra_width_on_sides = (rack_outer_width_height[0] - rack_inner_width_height[0]) / 2.0f;
    // CARDBOARD WIDTH 0.008 OUTER ANGLE IS OUTSIDE OF CARDBOARD;
    model_origin_offset_from_scene[0] = (bin_widths[0] - extra_width_on_sides) / 2.0f + 0.008;


    int reference_bin_no = 2;
    reference_bin_no--;

    model_origin_offset_from_scene[1] = bin_inner_heights_from_top[reference_bin_no] - bin_heights[0]/2.0f;


    for (int i = 0; i < 3; i++)
        shelf_details.bin_widths[i] = bin_widths[i];
    for (int i = 0; i < 2; i++)
        shelf_details.bin_heights[i] = bin_heights[i];
    for (int i = 0; i < 3; i++)
        shelf_details.bin_inner_heights_from_top[i] = bin_inner_heights_from_top[i];
    for (int i = 0; i < 2; i++)
        shelf_details.rack_inner_width_height[i] = rack_inner_width_height[i];
    for (int i = 0; i < 2; i++)
        shelf_details.rack_outer_width_height[i] = rack_outer_width_height[i];
    for (int i = 0; i < 2; i++)
        shelf_details.model_origin_offset_from_scene[i] = model_origin_offset_from_scene[i];
    for (int i = 0; i < 12*2; i++)
        shelf_details.bin_widths_heights[i] = bin_widths_heights[i];
    shelf_details.bin_depth = bin_depth;

    shelf_details.ptr_centroid_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ptr_centroid_cloud, *shelf_details.ptr_centroid_cloud);

    /*
    pcl::visualization::PCLVisualizer pcl_visualizer;
    pcl_visualizer.addCoordinateSystem(0.10f, 0.0, 0.0, 0.0, "COORD1");

    pcl_visualizer.addPointCloud(shelf_details.ptr_centroid_cloud, "CLOUD1");
    pcl_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "CLOUD1");

    while (1)		pcl_visualizer.spinOnce(40);
    */

}

#endif
