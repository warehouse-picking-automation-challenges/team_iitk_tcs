#include<iostream>
#include<Kinect.h>
#include<opencv2\opencv.hpp>
#include<opencv2\core\cuda.hpp>
#include<opencv2\cudawarping.hpp>
#include<opencv2\cudaimgproc.hpp>
#include<opencv2\highgui.hpp>

#include"kinect_routines.h"
#include"kinect_to_point_cloud.h"

#include<pcl\point_cloud.h>

#include<pcl\visualization\pcl_visualizer.h>
#include<pcl\visualization\cloud_viewer.h>

#include<pcl\filters\passthrough.h>
#include<pcl\io\pcd_io.h>
#include<pcl\kdtree\kdtree.h>

#include<pcl\filters\statistical_outlier_removal.h>
#include<pcl\filters\radius_outlier_removal.h>
#include<pcl\filters\approximate_voxel_grid.h>

#include<pcl\features\normal_3d.h>
#include<pcl\segmentation\sac_segmentation.h>
#include<pcl\filters\extract_indices.h>
#include<pcl\segmentation\supervoxel_clustering.h>
#include<pcl\features\principal_curvatures.h>

#include<pcl\registration\transformation_estimation_svd.h>
#include<pcl\registration\icp_nl.h>
#include<pcl\registration\icp.h>

#include<thread>
#include"Shelf_and_Model_details.h"
#include"SHELF_DIMENSIONS.h"


#define COLOR_IMAGE_WIDTH 640
#define COLOR_IMAGE_HEIGHT 480

using namespace KinectRoutines;
using namespace kinect_data_to_point_cloud;

typedef struct _MouseData{
	int event;
	int flag;
	cv::Point2i point;
	bool data_valid;
}MouseData;

//----- MIN MAX SCORE CALULATION
void compute_max_min_height_score(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_cloud, float* min_max_height, int model_no, cv::Mat& max_min_score);

//----- TRANSFORMATION OF CLOUD
void compute_corresponding_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_bin_points,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_transformed_bin_points);

void get_transformation_matrix_from_scene_to_origin(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_bin_points,
	Eigen::Affine3f& transformation);

void get_transformed_cloud(cv::Mat& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_transformed_cloud_RGB);

//-----CLOUD FILTERING

void filter_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud,
	bool filter_pass_through, bool filter_statistically, bool filter_radially, unsigned char* mask,
	float* field_limits, float std_dev, int nebrs);

void remove_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud, pcl::IndicesConstPtr& ptr_removed_point_indices);


void filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_pixel_cloud,
	bool filter_pass_through, bool filter_statistically, bool filter_radially, unsigned char* mask,
	float* field_limits, float std_dev, int nebrs);

void remove_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_pixel_cloud, pcl::IndicesConstPtr& ptr_removed_point_indices);

///------- GET POINTS FROM IMAGE

void get_points(cv::Mat& image, std::vector<cv::Point2i>& points);

void mouse_callback_handler(int event, int x, int y, int flag, void* userdata);

void left_button_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons);

void right_button_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons);

void middle_button_handler(cv::Mat& mask, std::vector<std::vector<cv::Point2i>>& polygons);

void mouse_movement_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons);


//------ICP REGISTRATION

bool icp_incremental_two_cloud_aligning(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned, Eigen::Affine3f& transformation);

bool icp_nonlinear_incremental_two_cloud_aligning(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned, Eigen::Affine3f& transformation);

void register_bin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, Eigen::Affine3f& transformation);

void register_shelf(SHELF_DETAILS& shelf_details, pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud, Eigen::Affine3f& transformation);

//---REGISTERED RGB IMAGE CLOUD

void construct_registred_image_RGB_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_cloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB);

void update_registered_image_and_mask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud,
	cv::Mat& image_registered_rgb, cv::Mat& image_mask);

//----- SHOW IMAGES ON OTHER THREAD

void multithread_imshow(cv::Mat& image, cv::Mat& mask);

///------------SHELF ALIGNMENT

void get_shelf_details(SHELF_DETAILS& shelf_details);


void get_transformation_from_scene_to_model(SHELF_DETAILS& shelf_details,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_hough_points_cloud,
	Eigen::Affine3f& transformation_scene_to_model);

void get_scene_cloud_and_image(cv::Mat& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud);

void get_hough_points(cv::Mat& image, pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_hough_points_cloud);

//------ PLANE_REMOVAL

void remove_bin_boundaries(SHELF_DETAILS& shelf_details,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, int bin_no,
	Eigen::Affine3f& transformation_scene_to_model);

//-----------READ HISTOGRAMS

unsigned int read_histograms_accumulated(std::string hist_file_path, std::vector<cv::Mat>& histograms);

void read_all_hostograms(int model_no, std::vector<cv::Mat>& vector_histograms);

void compute_depth_mean_var(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_mean_var, cv::Mat& mask);

void compute_depth_principle_curvature(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_principle_curvature, cv::Mat& mask);

void process_hsv_image_flat_gray_scale_partition(cv::Mat& image);

void compute_cloud_to_color_image_processed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud, cv::Mat& color_image);

//------BACK PROJECTIONS

void back_project_depth_histogram(cv::Mat& mtx_mean_var, cv::Mat& hist, cv::Mat& mask, cv::Mat& back_projection);

void back_project_depth_histogram_dense(pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud, cv::Mat& back_projection_sparse, cv::Mat& back_projection_dense);

void back_project_normals_gaussian_distribution(cv::Mat& hist, cv::Mat& mask,
	cv::Mat& mtx_mean_var,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud,
	cv::Mat& back_projection_normals_gausssian_distribution);

void back_project_principle_curvatures(cv::Mat& hist, cv::Mat& mask,
	cv::Mat&  mtx_principle_curvature,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud,
	cv::Mat& back_projection_curvature);

void back_project_partitioned_histogram_fuzzy_gaussian_gray_scale_flat_partition(cv::Mat& processed_image,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, cv::Mat& hist, std::vector<cv::Mat>& vector_backprojections);

//------------BACKPROJECTED CLOUDS

void get_back_projected_clouds(std::vector<cv::Mat>& vector_back_projections_and_sum,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& vector_ptr_backprojected_clouds_RGB);

void get_color_probability_image_and_cloud(cv::Mat& mtx_back_projection, cv::Mat& mtx_colored_probability,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_colored_probability_cloud_RGB);

//----DETECT OBJECT

void get_all_back_projections(std::vector<int>& bin_members,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud,
	std::vector<cv::Mat>& vector_features, std::vector<cv::Mat>& vector_masks,
	std::vector<cv::Mat>& vector_back_projections_and_sum);

bool detect_object(std::vector<int>& bin_members, cv::Mat& image,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB);

bool detect_object_using_backprojections_and_confusion_matrix(std::vector<int>& bin_members, cv::Mat& image,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB);

void get_confusion_matrix(std::vector<int>& bin_members,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	std::vector<std::vector<cv::Mat>>& vector_histograms,
	cv::Mat& confusion_matrix);

////---FILE PATHS

std::string scene_file_path = "E:\\ASHISH\\Amazon Picking Challenge\\Official Object Database\\new objects raw database\\pcd\\pc_";
std::string image_file_path = "E:\\ASHISH\\Amazon Picking Challenge\\Official Object Database\\new objects raw database\\image\\img_";

std::string bin_file_path = "E:\\ASHISH\\Amazon Picking Challenge\\Official Object Database\\3D Dataset\\shelf_models\\bin_";

const float PERCENT_MIN_HEIGHT = 0.10;

const float MODEL_HEIGHTS[] = {
	0.170f, 0.065f,
	0.200f, 0.030f,
	0.095f, 0.070f,
	0.200f, 0.070f,
	0.100f, 0.020f,
	0.180f, 0.080f,
	0.165f, 0.070f,
	0.130f, 0.030f,
	0.260f, 0.085f,
	0.190f, 0.030f,
	0.140f, 0.030f,
	0.180f, 0.035f,
	0.230f, 0.020f,
	0.150f, 0.025f,
	0.160f, 0.070f,
	0.095f, 0.020f,
	0.200f, 0.080f
};

/*
const float BIN_WIDTH = 0.34f;
const float BIN_HEIGHT = 0.30f;
const float BIN_DEPTH = 0.50f;

const float SIDE_WALL_REMOVAL_ERROR = 0.009f;
const float BASE_WALL_REMOVAL_ERROR = 0.003f;
const float DEPTH_WALL_REMOVAL_ERROR = 0.01;

const float BIN_CUTTING_CENTROID_ERROR = 0.05;
const float BIN_CUTTING_DEPTH_ERROR = 0.015f;

const float CENTROID_ERROR = 0.04;
const float HEIGHT_ERROR = 0.01;
const float DEPTH_ERROR = 0.01;

const float COORDINATE_OFFSET = 0.01;
const float BASE_PLANE_REMOVAL_THRESHOLD = 0.005f;
*/

const float BIN_WIDTH = 0.90f;
const float BIN_HEIGHT = 0.42f;
const float BIN_DEPTH = 0.50f;

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



void main(void)
{
	//-------GET SHELF DIMENSIONS
	SHELF_DETAILS shelf_details;
	get_shelf_details(shelf_details);
	
	//--------GET PRIMARY CLOUD FOR BIN IDENTIFICATION
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat image;

	get_scene_cloud_and_image(image,ptr_scene_cloud_RGB, ptr_scene_cloud);

	//--------GET RACK CORNERS
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_hough_points_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	get_hough_points(image,ptr_scene_cloud, ptr_hough_points_cloud);

	//--------GET SCENE TO SAVED MODEL TRANSFORMATION
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Affine3f transformation_scene_to_model;
	get_transformation_from_scene_to_model(shelf_details, ptr_scene_cloud, ptr_hough_points_cloud,transformation_scene_to_model);
	
    // input cloud of objects = ptr_scene_cloud_RGB (Registered cloud), RGB image = image
	//--------GET IMAGE AND CLOUD FROM CLOSER VIEW OF THE BIN
	get_scene_cloud_and_image(image, ptr_scene_cloud_RGB, ptr_scene_cloud);

	//--------GET TRANSFORMATION FROM NEW CLOUD TO OLD CLOUD COORDINATE SYSTEMS
	Eigen::Affine3f transformation_new_scene_to_old_scene = Eigen::Affine3f::Identity();
	transformation_scene_to_model = transformation_new_scene_to_old_scene * transformation_scene_to_model;
		
	//--------CONSTRUCT REGISTERED PIXEL RGB CLOUD FOR COLOR IMAGE PROCESSING
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_pixel_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	construct_registred_image_RGB_cloud(ptr_scene_cloud_RGB, ptr_pixel_cloud_RGB);

	//--------GET BIN CLOUD WITH REMOVED BIN BOUNDARIES EXCEPT THE BASE
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_bin_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	
    // input bin number and total objects and bin memebers
	int bin_no;
	std::cout << "ENTER BIN NO" << std::endl;
	std::cin >> bin_no;
	
	remove_bin_boundaries(shelf_details, ptr_scene_cloud_RGB,ptr_pixel_cloud_RGB,
		ptr_bin_cloud_RGB, bin_no, transformation_scene_to_model);

	int total_objects;
	std::cout << "ENTER NO OF OBJECTS" << std::endl;
	std::cin >> total_objects;
	std::cout << "ENTER MEMBER NO" << std::endl;

	std::vector<int> bin_members(total_objects);
	for (int i = 0; i < total_objects; i++)
		std::cin >> bin_members[i];
	
	detect_object(bin_members,image, ptr_bin_cloud_RGB, ptr_pixel_cloud_RGB);

	pcl::visualization::PCLVisualizer pcl_visualizer;

	pcl_visualizer.addCoordinateSystem(0.10, 0.0, 0.0, 0.0, "COORD");
	//pcl_visualizer.addPointCloud(ptr_transformed_cloud_RGB, "CLOUD");
	//pcl_visualizer.addPointCloud(ptr_scene_cloud_RGB, "CLOUD1");
	pcl_visualizer.addPointCloud(ptr_bin_cloud_RGB, "CLOUD1");

	for (int i = 0; i < ptr_centroid_cloud->size(); i++)
	{
		std::stringstream circle_name;
		circle_name << "circle" << i;
		pcl_visualizer.addSphere(ptr_centroid_cloud->at(i), .02, 0, 255, 0, circle_name.str());
	}

	while (1)
		pcl_visualizer.spinOnce(40);

	/*
	pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");

	while (1)
	{
	//	cloud_viewer.showCloud(ptr_scene_cloud_RGB);		Sleep(1000);
		cloud_viewer.showCloud(ptr_transformed_cloud_RGB);		Sleep(1000);
		cloud_viewer.showCloud(ptr_centroid_cloud);		Sleep(1000);
	}
	*/
}



//---READ HISTOGRAMS

unsigned int read_histograms_accumulated(std::string hist_file_path, std::vector<cv::Mat>& histograms)
{
	unsigned int number_of_hists;

	FILE* output_file = fopen(hist_file_path.c_str(), "rb");

	int dims;
	int *hist_size;

	fread((void*)&number_of_hists, sizeof(int), 1, output_file);
	fread((void*)&dims, sizeof(int), 1, output_file);

	hist_size = new int[dims];
	fread(hist_size, sizeof(int), dims, output_file);

	printf("%d %d ", number_of_hists, dims);

	for (int i = 0; i < dims; i++)
		printf("%d ", hist_size[i]);
	printf("\n");

	int no_of_elements = 1.0;

	for (int i = 0; i < dims; i++)
		no_of_elements *= hist_size[i];

	for (int i = 0; i < number_of_hists; i++)
	{
		cv::Mat histogram;
		histogram.create(dims, hist_size, CV_32FC1);


		fread(histogram.data, histogram.step[dims - 1], no_of_elements, output_file);
		histograms.push_back(histogram);

		histogram.release();
	}

	fclose(output_file);

	return number_of_hists;
}

void read_all_hostograms(int model_no,std::vector<cv::Mat>& vector_histograms)
{
	std::string model_depth_hist_file_path = "E:\\ASHISH\\Amazon Picking Challenge\\Official Object Database\\3D Dataset\\depth histograms\\DEPTH_HIST_";
	std::string model_color_hist_file_path = "E:\\ASHISH\\Amazon Picking Challenge\\Official Object Database\\RGB Dataset\\color histograms\\COLOR_HIST_";

	std::stringstream model_depth_hist_file_name;
	model_depth_hist_file_name << model_depth_hist_file_path.c_str() << model_names[model_no] << ".txt";

	std::stringstream model_color_hist_file_name;
	model_color_hist_file_name << model_color_hist_file_path.c_str() << model_names[model_no] << ".txt";

	std::cout << model_depth_hist_file_name.str() << std::endl;
	std::cout << model_color_hist_file_name.str() << std::endl;

	read_histograms_accumulated(model_depth_hist_file_name.str().c_str(), vector_histograms);
	read_histograms_accumulated(model_color_hist_file_name.str().c_str(), vector_histograms);


	for (int i = 0; i < vector_histograms.size()-1; i++)
	{
		cv::normalize(vector_histograms[i], vector_histograms[i], 0.0, 1.0, cv::NORM_MINMAX);
	}
	for (int i = 0; i<vector_histograms[vector_histograms.size() - 1].cols;i++)
	cv::normalize(vector_histograms[vector_histograms.size() - 1].col(i), vector_histograms[vector_histograms.size() - 1].col(i), 0.0, 1.0, cv::NORM_MINMAX);

	//for (int i = 0; i < vector_histograms.size();i++)
	//std::cout << vector_histograms[i] << std::endl << std::endl;

}

void compute_depth_mean_var(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_mean_var, cv::Mat& mask)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr ptr_scene_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

	normal_estimator.setRadiusSearch(0.02);
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
		float radii = 0.02;
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

			if (isnan<float>(dot_product))
				continue;

			float angle = 180.0f * acosf(dot_product) / 3.141f;

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

		float outlier_rate = 1.2f;

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

		float max_limit = 100;

		int invalid = 0;

		if (filtered_angles.size())
		{
			if (mean >= max_limit || var >= max_limit)
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

			//	printf("%f %f \n ", mean, var);
		}
	}
}

void compute_depth_principle_curvature(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud, cv::Mat& mtx_principle_curvature, cv::Mat& mask)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr ptr_scene_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;

	normal_estimator.setRadiusSearch(0.025);
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
	principle_curvature_estimation.setRadiusSearch(.03);
	principle_curvature_estimation.setInputCloud(ptr_scene_cloud);
	principle_curvature_estimation.compute(*ptr_principle_curvature_cloud);

	for (int i = 0; i < ptr_principle_curvature_cloud->size(); i++)
	{
		float k1 = ptr_principle_curvature_cloud->at(i).pc1;
		float k2 = ptr_principle_curvature_cloud->at(i).pc2;

		float a = k1*k2;
		float b = (k1 + k2) / 2;


		//printf("%f %f %f %f\n", k1, k2, a, b);

		if (k1 >= 1.0f || k2 >= 1.0f || isnan<float>(k1) || isnan<float>(k2))
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

	cv::merge(splitted, 3, image);

	cv::imshow("REGION IMAGE", region_image);
	cv::waitKey(10);
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

//------BACK PROJECTIONS

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

				//		std::cout << bin_location_x << " " << bin_location_y << " ";
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud, cv::Mat& back_projection_sparse, cv::Mat& back_projection_dense)
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

void back_project_normals_gaussian_distribution(cv::Mat& hist,cv::Mat& mask,
	cv::Mat& mtx_mean_var,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud, 
	cv::Mat& back_projection_normals_gausssian_distribution)
{
	cv::Mat back_projection_sparse;

	back_project_depth_histogram(mtx_mean_var, hist, mask, back_projection_sparse);
	back_project_depth_histogram_dense(kd_tree,ptr_bin_cloud, ptr_bin_voxel_cloud, back_projection_sparse, back_projection_normals_gausssian_distribution);
}

void back_project_principle_curvatures(cv::Mat& hist, cv::Mat& mask,
	cv::Mat&  mtx_principle_curvature,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud,
	cv::Mat& back_projection_curvature)
{
	cv::Mat back_projection_sparse;

	back_project_depth_histogram(mtx_principle_curvature, hist, mask, back_projection_sparse);
	back_project_depth_histogram_dense(kd_tree,ptr_bin_cloud, ptr_bin_voxel_cloud, back_projection_sparse, back_projection_curvature);
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

	const float resolution = 30.0f;  //degrees
	const float var = 0.50 * resolution;

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
					weight = std::expf(weight);
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
					weight = std::expf(weight);
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

		for (int i = 0; i < backprojections.size();i++)
		vector_backprojections.push_back(backprojections[i]);
}

//------------BACKPROJECTED CLOUDS

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

	imshow("PROBABILITY IMAGE", mtx_colored_probability);
	cv::waitKey(1);
	
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


//----DETECT OBJECT

void get_all_back_projections(std::vector<int>& bin_members,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	pcl::search::KdTree<pcl::PointXYZ>& kd_tree,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_voxel_cloud,
	std::vector<cv::Mat>& vector_features, std::vector<cv::Mat>& vector_masks,
	std::vector<cv::Mat>& vector_back_projections_and_sum)
{
	bool load_depth_histograms = true;
	bool load_color_histograms = true;
	bool back_project_truecolor_histogram = true;


	//----BIN MEMEBRS[0] CORRESPONDS TO TARGET OBJECT
	std::vector<cv::Mat> vector_histograms;
	read_all_hostograms(bin_members[0], vector_histograms);

	int back_projection_no = 0;

	vector_back_projections_and_sum.push_back(cv::Mat());
	back_project_normals_gaussian_distribution(vector_histograms[back_projection_no], vector_masks[back_projection_no],
		vector_features[back_projection_no], kd_tree, ptr_bin_cloud, ptr_bin_voxel_cloud, vector_back_projections_and_sum[back_projection_no]);
	back_projection_no++;

	vector_back_projections_and_sum.push_back(cv::Mat());
	back_project_principle_curvatures(vector_histograms[back_projection_no], vector_masks[back_projection_no],
		vector_features[back_projection_no], kd_tree, ptr_bin_cloud, ptr_bin_voxel_cloud, vector_back_projections_and_sum[back_projection_no]);
	back_projection_no++;

	back_project_partitioned_histogram_fuzzy_gaussian_gray_scale_flat_partition(vector_features[back_projection_no],
		ptr_bin_cloud_RGB, vector_histograms[back_projection_no], vector_back_projections_and_sum);
	back_projection_no++;

	//cv::Mat sum_backprojections = cv::Mat::zeros(ptr_bin_cloud_RGB->size(), 1, CV_32FC1);
	cv::Mat sum_backprojections = cv::Mat::ones(ptr_bin_cloud_RGB->size(), 1, CV_32FC1);

	bool sum_or_product = true;

	for (int j = 0; j < vector_back_projections_and_sum.size() - 1; j++)
	{
		if (sum_or_product)
		{
			if (j == 2)
			{
				if (back_project_truecolor_histogram)
					sum_backprojections += vector_back_projections_and_sum[j];
				else
					sum_backprojections += vector_back_projections_and_sum[j + 1];
				break;
			}
			sum_backprojections += vector_back_projections_and_sum[j];
		}
		else
		{
			if (j == 2)
			{
				if (back_project_truecolor_histogram)
					cv::multiply(sum_backprojections, vector_back_projections_and_sum[j], sum_backprojections);
				else
					cv::multiply(sum_backprojections, vector_back_projections_and_sum[j + 1], sum_backprojections);
				break;
			}
			cv::multiply(sum_backprojections, vector_back_projections_and_sum[j], sum_backprojections);
 		}
	}
	cv::normalize(sum_backprojections, sum_backprojections, 0.0, 1.0f, cv::NORM_MINMAX);
	vector_back_projections_and_sum.push_back(sum_backprojections);
}


bool detect_object(std::vector<int>& bin_members, cv::Mat& image,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB)
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

	std::vector<cv::Mat> vector_masks;
	std::vector<cv::Mat> vector_features;
	int feature_no = 0;
	int mask_no = 0;

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_depth_mean_var(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_depth_principle_curvature(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_cloud_to_color_image_processed(ptr_bin_cloud_RGB, vector_features[feature_no++]);

	std::vector<cv::Mat> vector_back_projections_and_sum;
	get_all_back_projections(bin_members, ptr_bin_cloud_RGB,
		ptr_bin_cloud, kd_tree, ptr_bin_voxel_cloud, vector_features, vector_masks,vector_back_projections_and_sum);
	
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vector_ptr_back_projected_clouds;
	get_back_projected_clouds(vector_back_projections_and_sum, ptr_bin_cloud_RGB, vector_ptr_back_projected_clouds);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_colored_probability_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Mat colored_probability_image;


	//std::cout << vector_back_projections_and_sum[4] << std::endl;
    //cv::threshold(vector_back_projections_and_sum[4], vector_back_projections_and_sum[4], 0.6, 1.0, CV_THRESH_BINARY);
	
	get_color_probability_image_and_cloud(vector_back_projections_and_sum[4],
		colored_probability_image,	ptr_bin_cloud_RGB, ptr_colored_probability_cloud_RGB);


	pcl::visualization::CloudViewer cloud_viewer("CLOUD");

	while (1)
	{
		cloud_viewer.showCloud(ptr_bin_cloud_RGB); Sleep(1000);
		for (int i = 0; i < vector_ptr_back_projected_clouds.size(); i++)
		{
			cloud_viewer.showCloud(vector_ptr_back_projected_clouds[i]); Sleep(1000);
		}
		cloud_viewer.showCloud(ptr_colored_probability_cloud_RGB); Sleep(1000);
	}
}



bool detect_object_using_backprojections_and_confusion_matrix(std::vector<int> bin_members ,cv::Mat& image,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_pixel_cloud_RGB)
{
	bool load_depth_histograms = true;
	bool load_color_histograms = true;
	bool project_truecolor_histogram = true;

	std::vector<std::vector<cv::Mat>> vector_histograms;

	//----BIN MEMEBRS[0] CORRESPONDS TO TARGET OBJECT
	int total_histograms =0;
	for (int i = 0; i < bin_members.size(); i++)
	{
		vector_histograms.push_back(std::vector<cv::Mat>());
		read_all_hostograms(bin_members[i], vector_histograms[total_histograms++]);
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*ptr_bin_cloud_RGB, *ptr_bin_cloud);

	cv::Mat confusion_matrix;
	get_confusion_matrix(bin_members, ptr_bin_cloud_RGB, ptr_bin_cloud, vector_histograms, confusion_matrix);

	cv::Mat posterior;
	posterior.create(ptr_bin_cloud_RGB->size(), 1, CV_32FC1);

	cv::Mat labels;
	labels.create(ptr_bin_cloud_RGB->size(), 1, CV_8UC1);
	labels = 0;

	std::cout << confusion_matrix << std::endl;

	if (bin_members.size() > 1)
	{
		double min_val;
		double max_val;
		cv::Point2i location_min;
		cv::Point2i location_max;

		for (int i = 0; i < posterior.rows; i++)
		{
			cv::minMaxLoc(confusion_matrix.row(i), NULL, &max_val, NULL, &location_max);
			posterior.at<float>(i) =
				1.0f - ((float*)(confusion_matrix.data + i*confusion_matrix.step[0] + location_max.x * confusion_matrix.step[1]))[0];
			std::cout << location_max << std::endl;

			if (location_max.x == 0)
			{
				posterior.at<float>(i) = 1.0f - posterior.at<float>(i);
				labels.at<unsigned char>(i) = location_max.x;
			}
		}
	}
	else
	{
		cv::threshold(confusion_matrix, posterior, 0.6, 1.0f, CV_THRESH_BINARY);
		for (int i = 0; i < posterior.rows; i++)
		{
			labels.at<unsigned char>(i) = 1-(unsigned char)posterior.at<float>(i);
		}
	}
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptr_classified_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (int i = 0; i < ptr_bin_cloud_RGB->size(); i++)
	{
		pcl::PointXYZRGBA point;
		point.x = ptr_bin_cloud_RGB->at(i).x;
		point.y = ptr_bin_cloud_RGB->at(i).y;
		point.z = ptr_bin_cloud_RGB->at(i).z;


		unsigned int class_obj = labels.at<unsigned char>(i);

		unsigned char colors[][3] = {
			{ 255, 0, 0 },
			{ 0, 255, 0 },
			{ 0, 0, 255 },
			{ 255, 255, 0 },
			{ 255, 0, 255 },
			{ 0, 255, 255 },
			{ 255, 255, 255 }
		};


		point.r = colors[class_obj][0];
		point.g = colors[class_obj][1];
		point.b = colors[class_obj][2];

		ptr_classified_cloud_RGB->push_back(point);

	}

	pcl::visualization::CloudViewer cloud_viewer("CLOUD VIEWER");

	while (1)
	{
		cloud_viewer.showCloud(ptr_bin_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_classified_cloud_RGB); Sleep(1000);

	}


}

void get_confusion_matrix(std::vector<int>& bin_members,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_bin_cloud,
	std::vector<std::vector<cv::Mat>>& vector_histograms,
	cv::Mat& confusion_matrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setDownsampleAllData(true);
	voxel_filter.setLeafSize(0.004, 0.004, 0.004);
	voxel_filter.setInputCloud(ptr_bin_cloud);
	voxel_filter.filter(*ptr_bin_voxel_cloud);

	pcl::search::KdTree<pcl::PointXYZ> kd_tree;
	kd_tree.setInputCloud(ptr_bin_voxel_cloud);

	std::vector<cv::Mat> vector_masks;
	std::vector<cv::Mat> vector_features;
	int feature_no = 0;
	int mask_no = 0;

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_depth_mean_var(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_depth_principle_curvature(ptr_bin_voxel_cloud, vector_features[feature_no++], vector_masks[mask_no++]);

	vector_masks.push_back(cv::Mat());
	vector_features.push_back(cv::Mat());
	compute_cloud_to_color_image_processed(ptr_bin_cloud_RGB, vector_features[feature_no++]);
	vector_masks[mask_no++] = 255 * cv::Mat::ones(ptr_bin_cloud_RGB->size(), 1, CV_8UC1);


	std::vector<std::vector<cv::Mat>> vector_backprojections;

	confusion_matrix.release();
	confusion_matrix.create(ptr_bin_cloud_RGB->size(), bin_members.size(), CV_32FC1);

	for (int i = 0; i < bin_members.size(); i++)
	{
		int back_projection_no = 0;
		vector_backprojections.push_back(std::vector<cv::Mat>());

		vector_backprojections[i].push_back(cv::Mat());
		back_project_normals_gaussian_distribution(vector_histograms[i][back_projection_no], vector_masks[back_projection_no],
			vector_features[back_projection_no], kd_tree, ptr_bin_cloud, ptr_bin_voxel_cloud, vector_backprojections[i][back_projection_no]);
		back_projection_no++;

		vector_backprojections[i].push_back(cv::Mat());
		back_project_principle_curvatures(vector_histograms[i][back_projection_no], vector_masks[back_projection_no],
			vector_features[back_projection_no], kd_tree, ptr_bin_cloud, ptr_bin_voxel_cloud, vector_backprojections[i][back_projection_no]);
		back_projection_no++;

		back_project_partitioned_histogram_fuzzy_gaussian_gray_scale_flat_partition(vector_features[back_projection_no],
			ptr_bin_cloud_RGB, vector_histograms[i][back_projection_no], vector_backprojections[i]);
		back_projection_no++;

		std::cout << "BACK PROJECTION SIZE" <<  vector_backprojections[i].size() << std::endl;

		cv::Mat sum_backprojections = cv::Mat::zeros(ptr_bin_cloud_RGB->size(), 1, CV_32FC1);
		for (int j = 0; j < vector_backprojections[i].size() - 1; j++)
		{
			if(j==2)sum_backprojections += vector_backprojections[i][j];
		}
		for (int j = 0; j < sum_backprojections.rows; j++)
			((float*)(confusion_matrix.data + j*confusion_matrix.step[0] + i*confusion_matrix.step[1]))[0] = sum_backprojections.at<float>(j);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_back_projection_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_back_projection_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < ptr_bin_cloud_RGB->size(); i++)
	{
		pcl::PointXYZRGB point;
		point.x = ptr_bin_cloud_RGB->at(i).x;
		point.y = ptr_bin_cloud_RGB->at(i).y;
		point.z = ptr_bin_cloud_RGB->at(i).z;
		point.r = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[0];
		point.g = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[0];
		point.b = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[0];

		ptr_back_projection_cloud1->push_back(point);
		point.r = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[1];
		point.g = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[1];
		point.b = 255.0 * ((float*)(confusion_matrix.data + i*confusion_matrix.step[0]))[1];
		ptr_back_projection_cloud2->push_back(point);
	}

	pcl::visualization::CloudViewer cloud_viewer("CLOUD");
	while (1)
	{
		cloud_viewer.showCloud(ptr_bin_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_back_projection_cloud1); Sleep(1000);
		cloud_viewer.showCloud(ptr_back_projection_cloud2); Sleep(1000);

	}

}


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

void get_transformation_from_scene_to_model(SHELF_DETAILS& shelf_details,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_hough_points_cloud,
	Eigen::Affine3f& transformation_scene_to_model)
{
	Eigen::Affine3f transformation_inverted_cloud = Eigen::Affine3f::Identity();
	transformation_inverted_cloud(1, 1) = -1;   // INVERT Y COORDINATE

	pcl::transformPointCloud(*ptr_scene_cloud, *ptr_scene_cloud, transformation_inverted_cloud);
	pcl::transformPointCloud(*ptr_hough_points_cloud, *ptr_hough_points_cloud, transformation_inverted_cloud);

	Eigen::Affine3f transformation_scene_to_origin;
	get_transformation_matrix_from_scene_to_origin(ptr_hough_points_cloud, transformation_scene_to_origin);

	pcl::transformPointCloud(*ptr_scene_cloud, *ptr_scene_cloud, transformation_scene_to_origin);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::stringstream bin_file_name;
	bin_file_name << bin_file_path.c_str() << "all0.pcd";
	pcl::io::loadPCDFile(bin_file_name.str().c_str(), *ptr_bin_cloud);

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

void get_scene_cloud_and_image(cv::Mat& image,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud)
{
	std::cout << "MODEL NAMES AND  NUMBER" << std::endl << std::endl;
	for (int i = 0; i < no_of_classes; i++)
		std::cout << model_names[i].c_str() << "---- >  " << i << std::endl;

	int scene_cloud_no = 1;
		
	std::cout << "ENTER SCENE CLOUD NUMBER" << std::endl << std::endl;
	std::cin >> scene_cloud_no;
	std::stringstream scene_file;
	std::stringstream image_file;
	scene_file << scene_file_path.c_str() << scene_cloud_no << ".pcd";
	//scene_file << scene_file_path.c_str() << model_names[model_no] << scene_cloud_no << ".pcd";
	image_file << image_file_path.c_str() << scene_cloud_no << ".jpg";

	image = cv::imread(image_file.str());
	//cv::imshow("IM", image); cv::waitKey(1);
	pcl::io::loadPCDFile(scene_file.str().c_str(), *ptr_scene_cloud_RGB);

	pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud);

}

void get_hough_points(cv::Mat& image,	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_scene_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr_hough_points_cloud)
{
	std::vector<cv::Point2i> points;
	get_points(image, points);
	
	for (int i = 0; i < 4; i++)
		ptr_hough_points_cloud->push_back(ptr_scene_cloud->at(points[i].x, points[i].y));

	for (int i = 0; i < 4; i++)
		printf("%f %f %f \n", ptr_hough_points_cloud->at(i).x, ptr_hough_points_cloud->at(i).y, ptr_hough_points_cloud->at(i).z);
}

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
	field_limits[5] = shelf_details.bin_depth - DEPTH_WALL_REMOVAL_ERROR,

	filter_point_cloud(ptr_bin_cloud_RGB, ptr_bin_cloud_RGB, ptr_pixel_cloud_RGB,
		true, false, false, NULL, field_limits, 1.4, 10);

	/*
	pcl::visualization::CloudViewer cloud_viewer("CLOUD");
	while (1)
	{
		cloud_viewer.showCloud(ptr_scene_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_bin_cloud_RGB); Sleep(1000);
	}
	*/
}


//----- SHOW IMAGES ON OTHER THREAD

void multithread_imshow(cv::Mat& image, cv::Mat& mask)
{
	while (1)
	{
		cv::imshow("REGISTERED_IMAGE", image);
		cv::imshow("REGISTERED_MASK", mask);
		cv::waitKey(10);
	}
}

//---REGISTERED RGB IMAGE CLOUD

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
	image_mask = 0;
	image_registered_rgb = 0;

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
		PCL_INFO("Iteration Nr. %d.\n", i);

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

	pcl::search::KdTree<pcl::PointXYZ, pcl::KdTreeFLANN<pcl::PointXYZ>>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ, pcl::KdTreeFLANN<pcl::PointXYZ>>);

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

void register_bin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_scene_cloud_RGB,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr_bin_cloud_RGB, Eigen::Affine3f& transformation)
{
	float field_limits_bin_cutting[] = {
		-BIN_CUTTING_CENTROID_ERROR, BIN_WIDTH + BIN_CUTTING_CENTROID_ERROR,
		-BIN_CUTTING_CENTROID_ERROR, BIN_HEIGHT + BIN_CUTTING_CENTROID_ERROR,
		//-BIN_CUTTING_DEPTH_ERROR,  BIN_DEPTH + BIN_CUTTING_DEPTH_ERROR
		-BIN_CUTTING_DEPTH_ERROR, BIN_CUTTING_DEPTH_ERROR
	};

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cropped_scene_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_pixel_cloud_RGB(NULL);

	filter_point_cloud(ptr_scene_cloud_RGB, ptr_cropped_scene_cloud_RGB, ptr_pixel_cloud_RGB,
		true, false, false, NULL, field_limits_bin_cutting, 1.4, 10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*ptr_cropped_scene_cloud_RGB, *ptr_scene_cloud);
	pcl::copyPointCloud(*ptr_bin_cloud_RGB, *ptr_bin_cloud);

	//Eigen::Affine3f transformation;
	//icp_nonlinear_incremental_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);
	icp_incremental_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);

	/*
	pcl::visualization::CloudViewer cloud_viewer("Cloud");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_aligned_cloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*ptr_scene_cloud_RGB, *ptr_aligned_cloud_RGB);
	pcl::transformPointCloud(*ptr_aligned_cloud_RGB, *ptr_aligned_cloud_RGB, transformation);
	while (1)
	{
		cloud_viewer.showCloud(ptr_scene_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_aligned_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_bin_cloud); Sleep(1000);
		cloud_viewer.showCloud(ptr_cropped_scene_cloud_RGB); Sleep(1000);
		cloud_viewer.showCloud(ptr_aligned_cloud); Sleep(1000);
	}
	*/

	//icp_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);
	pcl::transformPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud_RGB, transformation);
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
	cloud_viewer.showCloud(ptr_scene_cloud); Sleep(1000);
	cloud_viewer.showCloud(ptr_aligned_cloud); Sleep(1000);
	cloud_viewer.showCloud(ptr_bin_cloud); Sleep(1000);
	cloud_viewer.showCloud(ptr_cropped_scene_cloud); Sleep(1000);
		}
	*/

	//icp_two_cloud_aligning(ptr_scene_cloud, ptr_bin_cloud, ptr_aligned_cloud, transformation);
	//pcl::transformPointCloud(*ptr_scene_cloud_RGB, *ptr_scene_cloud_RGB, transformation);
}

//--------  CANDIADTES FOR MAXIMUM AND MINIMUM HEIGHTS

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

//-----CLOUD FILTERING

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

				printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());

				if (ptr_pixel_cloud && ptr_removed_point_indices->size())
				{
					printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
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

		printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());
		if (ptr_pixel_cloud && ptr_removed_point_indices->size())
		{
			printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
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
		printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());
		if (ptr_pixel_cloud && ptr_removed_point_indices->size())
		{
			printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
			remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
		}
	}
}

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

				printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());

				if (ptr_pixel_cloud && ptr_removed_point_indices->size())
				{
					printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
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

		printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());
		if (ptr_pixel_cloud && ptr_removed_point_indices->size())
		{
			printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
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
		printf("SOURCE FILTERED = %d  %d\n", source_cloud_ptr->size(), filtered_cloud_ptr->size());
		if (ptr_pixel_cloud && ptr_removed_point_indices->size())
		{
			printf("removed point indices = %d  %d\n", ptr_removed_point_indices->size(), ptr_pixel_cloud->size());
			remove_indices(ptr_pixel_cloud, ptr_removed_point_indices);
		}
	}
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


//------- GET CLICKED POINTS

void get_points(cv::Mat& image, std::vector<cv::Point2i>& points)
{
	MouseData mouse_data;
	cv::MouseCallback mouse_callback = mouse_callback_handler;

	cv::namedWindow("Image");
	cv::setMouseCallback("Image", mouse_callback, &mouse_data);


	cv::Mat display_image;
	image.copyTo(display_image);

	std::vector<std::vector<cv::Point2i>> polygons;

	mouse_data.data_valid = false;

	cv::Mat mask;
	mask.create(image.rows, image.cols, CV_8UC3);
	mask = 0;

	bool continue_with_same_image = true;
	bool single_polygon_completed = true;
	bool is_seed_point_available = false;
	int polygon_no = -1;

	while (continue_with_same_image)
	{

		while (!mouse_data.data_valid)
		{
			cv::imshow("Image", display_image);
			cv::waitKey(1);
		}


		switch (mouse_data.event)
		{
		case cv::EVENT_LBUTTONDOWN:
			image.copyTo(display_image);
			left_button_handler(mouse_data, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
			break;

		case cv::EVENT_RBUTTONDOWN:
			image.copyTo(display_image);
			right_button_handler(mouse_data, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
			break;
		case cv::EVENT_MBUTTONDOWN:
			continue_with_same_image = false;
			break;
		case cv::EVENT_MOUSEMOVE:
			image.copyTo(display_image);
			mouse_movement_handler(mouse_data, polygon_no, single_polygon_completed, is_seed_point_available, display_image, polygons);
			break;
		}

		mouse_data.data_valid = false;
	}

	points = polygons[0];

	cv::destroyWindow("Image");
	cv::waitKey(1);
}

void left_button_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons)
{
	if (single_polygon_completed)
	{
		std::vector<cv::Point2i> vector_point;
		polygons.push_back(vector_point);
		polygon_number++;
		single_polygon_completed = false;
		is_seed_point_available = false;
	}


	{
		cv::Scalar color(0, 255, 0);
		int thickness = 2;
		cv::Scalar color_circle(0, 0, 255);
		int point_circle_radii = 5;
		int circle_thickness = -1;

		{
			if (is_seed_point_available)
			{
				float polygon_termination_radii = 7.0;
				float x = polygons[polygon_number][0].x - mouse_data.point.x;
				float y = polygons[polygon_number][0].y - mouse_data.point.y;

				float distance = sqrtf(x*x + y*y);

				if (distance < polygon_termination_radii)
				{
					single_polygon_completed = true;
					printf("Polygon No %d Completed\n", polygon_number);
				}
				else
				{
					polygons[polygon_number].push_back(mouse_data.point);
				}
			}
			else
			{
				polygons[polygon_number].push_back(mouse_data.point);
				cv::circle(display_image, polygons[polygon_number][0], point_circle_radii, color_circle, circle_thickness);
				is_seed_point_available = true;
			}
		}



		if (single_polygon_completed)
		{
			for (int i = 0; i < polygons.size(); i++)
			{
				int polygon_size = polygons[i].size();
				for (int j = 0; j < polygon_size; j++)
				{
					cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
					cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
				}
			}
		}
		else
		{
			for (int i = 0; i < polygons.size() - 1; i++)
			{
				int polygon_size = polygons[i].size();
				for (int j = 0; j < polygon_size; j++)
				{
					cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
					cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
				}
			}

			int polygon_size = polygons[polygon_number].size();
			for (int j = 0; j < polygon_size - 1; j++)
			{
				cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
				cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
			}
			cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);

		}

	}
}

void right_button_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons)
{
	if (polygons.size())
	{
		if (polygons[polygon_number].size())
		{
			polygons[polygon_number].pop_back();
			single_polygon_completed = false;
		}

		if (!polygons[polygon_number].size())
		{
			polygons.pop_back();
			single_polygon_completed = true;
			polygon_number--;
			if (polygon_number > -1)
				is_seed_point_available = true;
		}

		cv::Scalar color(0, 255, 0);
		int thickness = 2;
		cv::Scalar color_circle(0, 0, 255);
		int point_circle_radii = 5;
		int circle_thickness = -1;

		if (single_polygon_completed)
		{
			for (int i = 0; i < polygons.size(); i++)
			{
				int polygon_size = polygons[i].size();
				for (int j = 0; j < polygon_size; j++)
				{
					cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
					cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
				}
			}
		}
		else
		{
			for (int i = 0; i < polygons.size() - 1; i++)
			{
				int polygon_size = polygons[i].size();
				for (int j = 0; j < polygon_size; j++)
				{
					cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
					cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
				}
			}

			int polygon_size = polygons[polygon_number].size();
			for (int j = 0; j < polygon_size - 1; j++)
			{
				cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
				cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
			}
			cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);

		}

	}
}

void middle_button_handler(cv::Mat& mask, std::vector<std::vector<cv::Point2i>>& polygons)
{
	cv::fillPoly(mask, polygons, cv::Scalar(255, 255, 255));
}

void mouse_movement_handler(MouseData& mouse_data, int& polygon_number, bool& single_polygon_completed, bool& is_seed_point_available,
	cv::Mat& display_image, std::vector<std::vector<cv::Point2i>>& polygons)
{

	cv::Scalar color(0, 255, 0);
	int thickness = 2;
	cv::Scalar color_circle(0, 0, 255);
	int point_circle_radii = 5;
	int circle_thickness = -1;

	if (single_polygon_completed)
	{
		for (int i = 0; i < polygons.size(); i++)
		{
			int polygon_size = polygons[i].size();
			for (int j = 0; j < polygon_size; j++)
			{
				cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
				cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
			}
		}
	}
	else
	{
		for (int i = 0; i < polygons.size() - 1; i++)
		{
			int polygon_size = polygons[i].size();
			for (int j = 0; j < polygon_size; j++)
			{
				cv::line(display_image, polygons[i][j%polygon_size], polygons[i][(j + 1) % polygon_size], color, thickness);
				cv::circle(display_image, polygons[i][j], point_circle_radii, color_circle, circle_thickness);
			}
		}

		int polygon_size = polygons[polygon_number].size();
		for (int j = 0; j < polygon_size - 1; j++)
		{
			cv::line(display_image, polygons[polygon_number][j], polygons[polygon_number][j + 1], color, thickness);
			cv::circle(display_image, polygons[polygon_number][j], point_circle_radii, color_circle, circle_thickness);
		}
		cv::line(display_image, polygons[polygon_number][polygon_size - 1], mouse_data.point, color, thickness);
		cv::circle(display_image, polygons[polygon_number][polygon_size - 1], point_circle_radii, color_circle, circle_thickness);
		cv::circle(display_image, mouse_data.point, point_circle_radii, color_circle, circle_thickness);

	}
}

void mouse_callback_handler(int event, int x, int y, int flag, void* userdata)
{
	MouseData *mouse_data = (MouseData*)userdata;
	mouse_data->event = event;
	mouse_data->point.x = x;
	mouse_data->point.y = y;
	mouse_data->flag = flag;
	mouse_data->data_valid = true;
}

