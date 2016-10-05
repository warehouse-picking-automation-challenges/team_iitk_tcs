#include "lines_rack_det/rackdetect.h"

RackDetect::RackDetect()
{
    ref_cloud = PointCloud<PointXYZRGBA>::Ptr (new PointCloud<PointXYZRGBA>);
    src_cloud = PointCloud<PointXYZRGBA>::Ptr (new PointCloud<PointXYZRGBA>);

    bin_corners = vector< vector<Eigen::Vector4d> > ((int) NO_BINS, vector<Eigen::Vector4d> (4));
    bin_centroids = vector<Eigen::Vector4d> ((int) NO_BINS);

    tf_bin_corners = vector< vector<Eigen::Vector4d> > ((int) NO_BINS, vector<Eigen::Vector4d> (4));
    tf_bin_centroids = vector<Eigen::Vector4d> ((int) NO_BINS);

    vertical_corners = vector<Eigen::Vector4d> (4);
    horizontal_corners = vector<Eigen::Vector4d> (6);

    rack_corners = vector<Eigen::Vector4d> (20);
    tf_rack_corners = vector<Eigen::Vector4d> (20);

    video_path = ros::package::getPath("lines_rack_det").append("/rack_detection.avi");
    video = VideoWriter (video_path.c_str(),CV_FOURCC('M','J','P','G'),2, Size(640,480),true);

    return;
}

bool RackDetect::inWorkspace(PointXYZRGBA point)
{
    vector<double> range_max(3), range_min(3);
    range_max[0] = 1.0;
    range_max[1] = 5.0;
    range_max[2] = 1.3;

    range_min[0] = -1.0;
    range_min[1] = -5.0;
    range_min[2] = 0.85;

    if(!isnan(point.x))// if it is a valid definite distance point
    {
        if(range_min[0] < point.x && point.x < range_max[0] &&
                range_min[1] < point.y && point.y < range_max[1] &&
                range_min[2] < point.z && point.z < range_max[2])
            return true;    // In workspace
        else
            return false;
    }
    else
    {
        return false; // In case depth information is not available
    }
}

// cloud: pointer to input point cloud containing rack within workspace of 2.0m
// Creates an image and point cloud of points within the workspace
void RackDetect::createRackImageCloud(PointCloud<PointXYZRGBA>::Ptr cloud)
{
    this->ref_image.release();
    this->ref_image.create(cloud->height,cloud->width,CV_8UC3);

    this->ref_cloud->width = cloud->width;
    this->ref_cloud->height = cloud->height;
    this->ref_cloud->is_dense = cloud->is_dense;
    this->ref_cloud->points.resize(cloud->width*cloud->height);

    const float bad_point = std::numeric_limits<float>::quiet_NaN();

    for(int i=0; i<cloud->height; i++)
        for(int j=0; j<cloud->width; j++)
        {
            PointXYZRGBA pt = cloud->at(j,i);

            if(this->inWorkspace(pt))
            {
                this->ref_cloud->at(j,i) = cloud->at(j,i);

                Eigen::Vector3i v = pt.getRGBVector3i();
                Vec3b color(v(2),v(1),v(0));
                this->ref_image.at<Vec3b>(i,j) = color;
            }
            else
            {
                this->ref_cloud->at(j,i).x = bad_point;
                this->ref_cloud->at(j,i).y = bad_point;
                this->ref_cloud->at(j,i).z = bad_point;

                this->ref_cloud->at(j,i).rgba = (uint32_t) 0;

                //Point is not in workspace, make the color black
                Vec3b color(0,0,0);
                this->ref_image.at<Vec3b>(i,j) = color;
            }
        }



    return;
}

void RackDetect::depthEdgeDetect(Mat &depthEdgeImage, int &thresh)
{
    NormalEstimationOMP<PointXYZRGBA, Normal> ne;
    ne.setInputCloud (this->ref_cloud);
    ne.setNumberOfThreads(8);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<PointXYZRGBA> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setKSearch(50); //setRadiusSearch( 0.01 );

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
    // Compute the features
    ne.compute ( *cloud_normals );

//    pcl::visualization::PCLVisualizer viewer("Normals viewer");
//    viewer.setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(this->ref_cloud);
//    viewer.addPointCloud<pcl::PointXYZRGBA> (this->ref_cloud, rgb, "sample cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//    viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (this->ref_cloud, cloud_normals, 10, 0.05, "normals");
//    viewer.addCoordinateSystem (0.1);
//    viewer.initCameraParameters ();

//    while(!viewer.wasStopped())
//        viewer.spinOnce();
//    viewer.close();

    int h = cloud_normals->height;
    int w = cloud_normals->width;

    //initialize a zero valued mat image
    depthEdgeImage = cv::Mat::zeros(h, w, CV_8UC1);

#pragma omp for
    // computing surface normal edges
    for(int j = 1 ; j < cloud_normals->width - 1 ; j++ )
    {
        for(int i = 1; i < cloud_normals->height - 1 ; i++ )
        {
            Normal pt;

            // accessing point cloud is different than normal
            // here (col, row) comes instead of (row,col)
            pt = cloud_normals->at(j,i);

            double x_center = pt.normal_x;
            double y_center = pt.normal_y;
            double z_center = pt.normal_z;

            // initialize dot product and count value to zero
            double dotProduct = 0.0;
            int noCount = 0;

            // process only if it is valid pt and the normal vector with 40deg of z-axis of kinect
            if(!isnan(x_center) && fabs(z_center)>0.75)
            {
                for( int i_inner = -1 ; i_inner <= 1 ; i_inner++ )
                {
                    for( int j_inner = -1 ; j_inner <= 1 ; j_inner++ )
                    {
                        if(!(i_inner == 0 && j_inner == 0))
//                        {
//                            // skip dot product with self
//                        }
//                        else
                        {
                            int x_index = j + j_inner;
                            int y_index = i + i_inner;
                            Normal ptNeigh = cloud_normals->at(x_index,y_index);

                            double x = ptNeigh.normal_x;
                            double y = ptNeigh.normal_y;
                            double z = ptNeigh.normal_z;

                            //checking condition for NAN
                            if(!isnan(x))
                            {
                                //compute dot product with 8 neighbourhood if the have a valid value
                                dotProduct = dotProduct + ( (x_center * x) + (y_center * y) + (z_center * z) );
                                noCount ++ ;
                            }
                        }
                    }
                }

                if(noCount > 1)
                    noCount = 8;
                dotProduct = dotProduct/ (float) noCount ;

                if( dotProduct < (double) thresh/(double)100 && dotProduct > 0.0 )
                {
                    depthEdgeImage.at<uchar>(i,j) = (char) 255;
                }
            }
        }
    }

    int dilation_type;
//    { dilation_type = MORPH_RECT; }
//    { dilation_type = MORPH_CROSS; }
    { dilation_type = MORPH_ELLIPSE; }
    int dilation_size = 2;
    Mat element = getStructuringElement( dilation_type,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    dilate(depthEdgeImage, depthEdgeImage, element);

    return;
}
// Function detects edges using 3d normal dot products
// All possible lines are detected using Hough transform
void RackDetect::getHoughLines(Mat &image, vector<Vec4i> &lines)
{
    Mat gray, dst;
//    cvtColor(image,gray,CV_BGR2GRAY);
//    int low = 60, high = 155;// canny edge detector parameter low and high values
//    int minLineLength = 360, threshold = 57;

    int minLineLength = 250, threshold = 100;
    int dp_th = 90;// Threshold for dot product, dp_th=cos(theta)*100

#if(DEBUG_MODE)
    namedWindow("detected lines");
//    createTrackbar("Edge low","detected lines",&low,255);
//    createTrackbar("Edge High","detected lines",&high,255);
    createTrackbar("Min line length","detected lines",&minLineLength,500);
    createTrackbar("Threshold","detected lines",&threshold,300);
    createTrackbar("dot prod Thres","detected lines",&dp_th,99);
#endif
    char c='m';
//    while(c!='q')
    {
//        Canny(gray, dst, low, high, 3);
        this->depthEdgeDetect(dst, dp_th);//dst is the edge detection image
        cvtColor(dst, this->lines_image, CV_GRAY2BGR);

#if 0
//        vector<Vec2f> lines;
        HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );
//        HoughLines(dst, lines, 1, CV_PI/180, threshold, minLineLength, 20);

        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( this->lines_image, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
        }
#else
        HoughLinesP(dst, lines, 1, CV_PI/180, threshold, minLineLength, 20 );        
#endif

#if(DEBUG_MODE)
        // Draw the lines in red color on to the image
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            line( this->lines_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
        }
//        while(c!='q' && c!=27)
        {
            imshow("edge detected", dst);
            c = (char) waitKey(1);
            imshow("detected lines",this->lines_image);
            c= (char) waitKey(1);
        }

#endif
    }

    return;
}

void RackDetect::splitVerticalHorizontal(vector<Vec4i> &all, vector<Vec4i> &vertical, vector<Vec4i> &horizontal)
{
    // Splits all of the lines into horizontal and veritcal lines.
    // Horizontal lines first point l(0,1) will correspond to left end and l(2,3) corresponds to right end
#if(DEBUG_MODE)
    cout << "Splitting into horizontal and vertical lines of total: " << all.size() << " lines" << endl;
    cout << "Length of lines: \n";
#endif
//    double min_length_lines = 0.5;
//    double dot_prod_threh = 0.5;

    for(int i=0; i<all.size(); i++)
    {
//#if(DEBUG_MODE)
//            cout << "line " << i << ": ";
//#endif
        Vec4i l = all[i];
        int col1 = l(0), col2 = l(2);
        int row1 = l(1), row2 = l(3);

//        Vec2i pt1_pxl(row1,col1), pt2_pxl(row2,col2);
//        PointXYZ pt1, pt2;

//        bool b1 = this->point2Dto3D(pt1_pxl, pt1);
//        bool b2 = this->point2Dto3D(pt2_pxl, pt2);
//        if(b1 && b2)
//        {

//            double d = fabs(sqrt(pow((pt1.x-pt2.x),2)+pow((pt1.y-pt2.y),2)+pow((pt1.z-pt2.z),2)));
//            Eigen::Vector3d v(pt1.x-pt2.x, pt1.y-pt2.y, pt1.z-pt2.z);
//            v.normalize();


//            if(d > min_length_lines && fabs(v(0)) < dot_prod_threh)// vertical line is orthogonal to x-axis of kinect
//            {
//                cout << d << ",vt dot prod: " << fabs(v(0)) << endl;
//                // Vertical line lower row should come first and then higher row
//                if(l(1) > l(3))// if row1 > row2 interchange them
//                {
//                    int tmp;
//                    tmp = l(1);    l(1) = l(3);    l(3) = tmp;
//                    tmp = l(0);    l(0) = l(2);    l(2) = tmp;
//                }
//                vertical.push_back(l);
//            }
//            else if(d > min_length_lines && fabs(v(1)) < dot_prod_threh)// horizontal line is orthogonal to y-axis of kinect
//            {
//                cout << d << ",ht dot prod: " << fabs(v(1)) << endl;
//                // Horizontal line lower col should come first and then higher col
//                if(l(0) > l(2))// if col1 > col2 interchange them
//                {
//                    int tmp;
//                    tmp = l(1);    l(1) = l(3);    l(3) = tmp;
//                    tmp = l(0);    l(0) = l(2);    l(2) = tmp;
//                }
//                horizontal.push_back(l);
//            }
//        }

        double row_dist = fabs(row1-row2), col_dist = fabs(col1-col2);
        Eigen::Vector2d pv(row_dist,col_dist);
        pv.normalize();
        if(pv(1)<0.5 && row_dist > 250)// for vertical line dot product of pv with row vector should be orthogonal
        {
//            cout << "Vt dist: " << row_dist << endl;
            // Vertical line lower row should come first and then higher row
            if(l(1) > l(3))// if row1 > row2 interchange them
            {
                int tmp;
                tmp = l(1);    l(1) = l(3);    l(3) = tmp;
                tmp = l(0);    l(0) = l(2);    l(2) = tmp;
            }
            vertical.push_back(l);
        }
        else if(pv(0)<0.5 && col_dist > 250)// for horizontal line dot product of pv with column vector should be orthogonal
        {
//            cout << "Ht dist: " << col_dist << endl;
            // Horizontal line lower col should come first and then higher col
            if(l(0) > l(2))// if col1 > col2 interchange them
            {
                int tmp;
                tmp = l(1);    l(1) = l(3);    l(3) = tmp;
                tmp = l(0);    l(0) = l(2);    l(2) = tmp;
            }
            horizontal.push_back(l);
        }


    }
    return;
}

// Function to find left most and right most clustered vertical lines
void RackDetect::getCornerVerticals(vector<Vec4i> &verticals, vector<Vec4i> &cluster_v)
{
    // This function first gets the left most and right most vertical line
    // Then cluster of lines around the left most and right most vertical line is formed
    int ll_idx = 0, rl_idx = 0;
    int ll_col = 640, rl_col = 0;
    // get the left most and right most vertical lines col index
    for(int i=0; i<verticals.size(); i++)
    {
        Vec4i l = verticals[i];
        int col1 = l(0), col2 = l(2);
//        int row1 = l(1), row2 = l(3);
        if(col1 < ll_col || col2 < ll_col)
        {
            ll_idx = i;
            ll_col = (col1 < col2) ? col1: col2;
        }
        else if(col1 > rl_col || col2 > rl_col)
        {
            rl_idx = i;
            rl_col = (col1 > col2) ? col1: col2;
        }
//        cout << "left M: " << ll_col << " ,right M: " << rl_col << endl;
    }

    // Cluster lines around left most and right most vertical lines within a kernel of distance 5 cm
    int col1, col2, row1, row2;
    Vec2i pt_pxl;

    int sum_col1, sum_col2, sum_row1, sum_row2, count;
    PointXYZ pt1, pt2;
    // cluster lines around the left most vertical lines
    Vec4i left_v = verticals[ll_idx];
    col1 = left_v(0), col2 = left_v(2);
    row1 = left_v(1), row2 = left_v(3);

    pt_pxl(0) = row1;
    pt_pxl(1) = col1;
    this->point2Dto3D(pt_pxl, pt1);

    pt_pxl(0) = row2;
    pt_pxl(1) = col2;
    this->point2Dto3D(pt_pxl, pt2);

    sum_col1 = col1, sum_col2 = col2, sum_row1 = row1, sum_row2 = row2;
    count = 1;
    for(int j=0; j<verticals.size(); j++)
    {
        Vec4i l = verticals[j];
        col1 = l(0), col2 = l(2);
        row1 = l(1), row2 = l(3);

//        PointXYZ tmp_pt1, tmp_pt2;
//        pt_pxl(0) = row1;
//        pt_pxl(1) = col1;
//        this->point2Dto3D(pt_pxl, tmp_pt1);

//        pt_pxl(0) = row2;
//        pt_pxl(1) = col2;
//        this->point2Dto3D(pt_pxl, tmp_pt2);

        //if any other vertical line is within 5cm take into the cluster
//        double cluster_dist = 0.05;
//        if(tmp_pt1.x < pt1.x+cluster_dist)
//        {
//            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
//            count++;
//        }
//        else if(tmp_pt2.x < pt2.x+cluster_dist)
//        {
//            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
//            count++;
//        }

        //if any other vertical line is within -5 to +10 pixel range then take into the cluster
        int cluster_k = 50;
        if(left_v(0) -cluster_k < col1 && col1 < left_v(0) + cluster_k)
        {
            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
            count++;
        }
        else if(left_v(2) -cluster_k < col2 && col2 < left_v(2) + cluster_k)
        {
            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
            count++;
        }
    }
    Vec4i left_line(sum_col1/count, sum_row1/count, sum_col2/count, sum_row2/count);
    cluster_v.push_back(left_line);

    // cluster lines around the right most vertical lines with a kernel of distance 5 cm
    Vec4i right_v = verticals[rl_idx];
    col1 = right_v(0), col2 = right_v(2);
    row1 = right_v(1), row2 = right_v(3);

    pt_pxl(0) = row1;
    pt_pxl(1) = col1;
    this->point2Dto3D(pt_pxl, pt1);

    pt_pxl(0) = row2;
    pt_pxl(1) = col2;
    this->point2Dto3D(pt_pxl, pt2);

    sum_col1 = col1, sum_col2 = col2, sum_row1 = row1, sum_row2 = row2;
    count = 1;
    for(int j=0; j<verticals.size(); j++)
    {
        Vec4i l = verticals[j];
        col1 = l(0), col2 = l(2);
        row1 = l(1), row2 = l(3);

//        PointXYZ tmp_pt1, tmp_pt2;
//        pt_pxl(0) = row1;
//        pt_pxl(1) = col1;
//        this->point2Dto3D(pt_pxl, tmp_pt1);

//        pt_pxl(0) = row2;
//        pt_pxl(1) = col2;
//        this->point2Dto3D(pt_pxl, tmp_pt2);

        //if any other vertical line is within 5 cm take into the cluster
//        double cluster_dist = 0.05;
//        if(tmp_pt1.x > pt1.x-cluster_dist)
//        {
//            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
//            count++;
//        }
//        else if(tmp_pt2.x > pt2.x-cluster_dist)
//        {
//            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
//            count++;
//        }

        //if any other vertical line is within -10 to +5 pixel range then take into the cluster
        if(right_v(0)-10 < col1 && col1 < right_v(0)+5)
        {
            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
            count++;
        }
        else if(right_v(2)-10 < col2 && col2 < right_v(2)+5)
        {
            sum_col1+=col1; sum_col2+=col2; sum_row1+=row1; sum_row2+=row2;
            count++;
        }
    }

    Vec4i right_line(sum_col1/count, sum_row1/count, sum_col2/count, sum_row2/count);
    cluster_v.push_back(right_line);

    return;
}

void RackDetect::getHorizontalClusters(vector<Vec4i> &horizontals, vector<Vec4i> &cluster_h)
{
    Mat img1, img2, img3;
    this->ref_image.copyTo(img1);
    this->ref_image.copyTo(img2);
    this->ref_image.copyTo(img3);

    // Cluster all the horizontal lines which are within +-20 row index difference
    int kernel_sz = 25;
    vector<Vec4i> horiz_clusters;
    vector<int> h_flag(horizontals.size(),0);
    int cluster_n = 0;
    for(int i=0; i<horizontals.size(); i++)
    {
        if(h_flag[i]==0)
        {
            cluster_n++;
            cout << "Cluster " << cluster_n << ": " ;
            Vec4i lc = horizontals[i];
            int col1_c = lc(0), col2_c = lc(2);
            int row1_c = lc(1), row2_c = lc(3);

            cout << "[" << row1_c << "," << col1_c << "][" << row2_c << "," << col2_c << "] ";
            h_flag[i] = 1;
            int count = 1;
            for(int j=i+1; j<horizontals.size(); j++)
            {
                if(h_flag[j] == 0)
                {
                    Vec4i l = horizontals[j];
                    int col1 = l(0), col2 = l(2);
                    int row1 = l(1), row2 = l(3);
//                     line will be in the cluster if row index is within +-20 index diff
                    if(row1 < lc(1)+kernel_sz && row1 > lc(1)-kernel_sz)
                    {
                        cout << "[" << row1 << "," << col1 << "][" << row2 << "," << col2 << "] ";
//                        if(row1 < row1_c+kernel_sz)
//                            cout << row1 << "<" << row1_c+kernel_sz << " ";
//                        else if(row1 > row1_c-kernel_sz)
//                            cout << row1 << ">" << row1_c-kernel_sz << " ";

                        row1_c = (row1_c+row1);
                        row2_c = (row2_c+row2);
                        col1_c = (col1_c+col1);
                        col2_c = (col2_c+col2);
                        h_flag[j] = 1;
                        count++;
                    }
                    else if(row2 < lc(3)+kernel_sz && row2 > lc(3)-kernel_sz)
                    {
                        cout << "[" << row1 << "," << col1 << "][" << row2 << "," << col2 << "] ";
//                        if(row2 < row2_c+kernel_sz)
//                            cout << row2 << "<" << row2_c+kernel_sz << " ";
//                        else if(row2 > row1_c-kernel_sz)
//                            cout << row2 << ">" << row2_c-kernel_sz << " ";

                        row1_c = (row1_c+row1);
                        row2_c = (row2_c+row2);
                        col1_c = (col1_c+col1);
                        col2_c = (col2_c+col2);
                        h_flag[j] = 1;
                        count++;
                    }
                }
            }

            Vec4i nl;
            nl(0) = col1_c/count; nl(2) = col2_c/count;
            nl(1) = row1_c/count; nl(3) = row2_c/count;
            horiz_clusters.push_back(nl);
//            cout << endl << "Cluster avg: [" << nl(1) << "," << nl(0) << "][" << nl(3) << "," << nl(2) << "]" << endl;
        }
    }
    cout << "Level 1 clusters: " << horiz_clusters.size() << endl;
    // Draw the two vertical lines in red color on to the image
    for( size_t i = 0; i < horiz_clusters.size(); i++ )
    {
        Vec4i l = horiz_clusters[i];
        line( img1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
#if(DEBUG_MODE)
    imshow("Level1 horiz cluster", img1);
    char c='m';
//    while(c!='q' && c!=27)
    {
        imshow("Level1 horiz cluster", img1);
        c = (char) waitKey(1);
    }
#endif

    // rearrange horiz_clusters in ascending order of row
    for(int i=0; i<horiz_clusters.size(); i++)
    {
        Vec4i l_tmp = horiz_clusters[i];
        for(int j=i+1; j<horiz_clusters.size(); j++)
        {
            Vec4i l_nxt = horiz_clusters[j];
            if(l_nxt(1) < l_tmp(1))
            {
                horiz_clusters[j] = l_tmp;
                l_tmp = l_nxt;
            }
        }
        horiz_clusters[i] = l_tmp;
    }
#if(DEBUG_MODE)
    // Remove the clustered lines which are below Y_LIMIT_MIN and above Y_LIMIT_MAX wrt y-axis of kinect
    cout << endl << "Horizontal lines within the 3 row block:" << endl;
#endif

    vector<Vec4i> tmp_clusters;
    tmp_clusters.assign(horiz_clusters.begin(),horiz_clusters.end());
    horiz_clusters.clear();
    for(int i=0; i<tmp_clusters.size(); i++)
    {
        Vec4i l = tmp_clusters[i];

        int col1 = l(0), col2 = l(2);
        int row1 = l(1), row2 = l(3);

        Vec2i pt1_pxl(row1,col1), pt2_pxl(row2,col2);
        PointXYZ pt1, pt2;

        this->point2Dto3D(pt1_pxl, pt1);
        this->point2Dto3D(pt2_pxl, pt2);

        if(pt1.y > Y_LIMIT_MIN && pt1.y < Y_LIMIT_MAX)
        {
            horiz_clusters.push_back(l);
        }
    }

#if(DEBUG_MODE)
//    cout << "Processing consecutive combine of horizontal lines within a distance of 10cm" << endl;
//    cout << "Before consecutive combine" << endl;
//    for(int i=0; i<horiz_clusters.size(); i++)
//    {
//        cout << "[" << horiz_clusters[i](1) << " " << horiz_clusters[i](0) << "]"
//             << "[" << horiz_clusters[i](3) << " " << horiz_clusters[i](2) << "]" << endl;
//    }
//    cout << "Processing consecutive combine for " << horiz_clusters.size() << " horizontal lines" << endl;
#endif

    // Combine consecutive horizontal lines which are within distance of 10cm wrt y-axis of kinect
    double cst_cmb_dist = 0.075;
    tmp_clusters.assign(horiz_clusters.begin(), horiz_clusters.end());
    horiz_clusters.clear();
    for(int i=0; i<tmp_clusters.size(); i++)
    {
        Vec4i l1 = tmp_clusters[i];
#if(DEBUG_MODE)
        cout << i << ": [" << tmp_clusters[i](1) << " " << tmp_clusters[i](0) << "]"
             << "[" << tmp_clusters[i](3) << " " << tmp_clusters[i](2) << "]" << endl;
#endif
        if(i != tmp_clusters.size()-1)
        {
            Vec4i l2 = tmp_clusters[i+1];
#if(DEBUG_MODE)
            cout << i+1 << ": [" << tmp_clusters[i+1](1) << " " << tmp_clusters[i+1](0) << "]"
                 << "[" << tmp_clusters[i+1](3) << " " << tmp_clusters[i+1](2) << "]" << endl;
#endif
            Vec2i pt11_pxl(l1(1),l1(0)), pt12_pxl(l1(3),l1(2));;
            PointXYZ pt11, pt12;
            this->point2Dto3D(pt11_pxl, pt11);
            this->point2Dto3D(pt12_pxl, pt12);

            Vec2i pt21_pxl(l2(1),l2(0)), pt22_pxl(l2(3),l2(2));
            PointXYZ pt21, pt22;
            this->point2Dto3D(pt21_pxl, pt21);
            this->point2Dto3D(pt22_pxl, pt22);


            if(fabs(pt11.y-pt21.y) < cst_cmb_dist || fabs(pt12.y-pt22.y) < cst_cmb_dist)
//            if(fabs())
            {
                int s_c1 = (l1(0)+l2(0))/2;
                int s_r1 = (l1(1)+l2(1))/2;
                int s_c2 = (l1(2)+l2(2))/2;
                int s_r2 = (l1(3)+l2(3))/2;

                Vec4i cl(s_c1,s_r1,s_c2,s_r2);
                horiz_clusters.push_back(cl);
#if(DEBUG_MODE)
                cout << "Combined: [" << s_r1 << " " << s_c1 << "]"
                     << "[" << s_r2 << " " << s_c2 << "]" << endl << endl;
#endif
                // i++ to skip the index since i+1 has been already part of the cluster
                i++;
            }
            else
            {
                horiz_clusters.push_back(l1);
#if(DEBUG_MODE)
                cout << "Retain: [" << l1(1) << " " << l1(0) << "][" << l1(3) << " " << l1(2) << "]" << endl << endl;
#endif
            }
        }
        else
        {
            horiz_clusters.push_back(l1);
#if(DEBUG_MODE)
            cout << "Retain: [" << l1(1) << " " << l1(0) << "][" << l1(3) << " " << l1(2) << "]" << endl << endl;
#endif
        }
    }

    for(int i=0; i<horiz_clusters.size(); i++)
        cluster_h.push_back(horiz_clusters[i]);
#if(DEBUG_MODE)
    cout << endl;
#endif
}

void RackDetect::getVerHorLines(vector<Vec4i> &lines, vector<Vec4i> &filter_vertical, vector<Vec4i> &filter_horizontal)
{
    Mat img, img2, img3;
    this->ref_image.copyTo(img);
    this->ref_image.copyTo(img2);
    this->ref_image.copyTo(img3);

    // split lines into horizontal and vertical lines of length greater than 0.5m
    vector<Vec4i> fv, fh;
    this->splitVerticalHorizontal(lines,fv,fh);
#if(DEBUG_MODE)
    cout << "Split Verticals: " << fv.size() << endl;
    cout << "Split horizontals: " << fh.size() << endl;
#endif

    for( size_t i = 0; i < fv.size(); i++ )
    {
        Vec4i l = fv[i];
        line( img2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    for( size_t i = 0; i < fh.size(); i++ )
    {
        Vec4i l = fh[i];
        line( img3, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
    }
    static int img2_c = 0, img3_c=0;
//    this->video.write(img2);
//    this->video.write(img3);
    char s[200];
    sprintf(s,"/home/ilab/tcs/apc_ws/src/apc_project/lines_rack_det/data/split_verticals_%d.jpg",img2_c);
    cv::imwrite(s, img2);
    img2_c++;
    sprintf(s,"/home/ilab/tcs/apc_ws/src/apc_project/lines_rack_det/data/split_horizontals_%d.jpg",img3_c);
    cv::imwrite(s, img3);
    img3_c++;

    char c2 = 'm';
#if(DEBUG_MODE)
    while(c2!='q' && c2!=27)
    {
        imshow("All V lines ", img2);
        imshow("All H lines ", img3);
        c2 = (char) waitKey(1);
    }
#endif

    if(fv.size() < 2 || fh.size() < 3)
        return;

    this->getCornerVerticals(fv,filter_vertical);
#if(DEBUG_MODE)
    for(int i=0; i<filter_vertical.size(); i++)
        cout <<"["<<filter_vertical[i](1)<<" "<<filter_vertical[i](0)<<"] ["<<filter_vertical[i](3)<<" "<<filter_vertical[i](2)<<"]"<<endl;
#endif

    // Draw the two vertical lines in red color on to the image
    for( size_t i = 0; i < filter_vertical.size(); i++ )
    {
        Vec4i l = filter_vertical[i];
        line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
    }
#if(DEBUG_MODE)
    imshow("VH lines cluster", img);
    waitKey(1);
    char c = (char) waitKey(1);
    char c='m';
    while(c!='q' && c!=27)
    {
        imshow("VH lines cluster", img);
        c = (char) waitKey(1);
    }
#endif

    // Cluster out the horizontal lines
    vector<Vec4i> horiz_clusters;
    this->getHorizontalClusters(fh, horiz_clusters);

    // Draw the lines in red color on to the image
    for( size_t i = 0; i < horiz_clusters.size(); i++ )
    {
        filter_horizontal.push_back(horiz_clusters[i]);
        Vec4i l = filter_horizontal[i];
        line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
    }
    static int img_c = 0;
//    this->video.write(img);
    sprintf(s,"/home/ilab/tcs/apc_ws/src/apc_project/lines_rack_det/data/clusters_%d.jpg",img_c);
    cv::imwrite(s, img);
    img_c++;
//#if(DEBUG_MODE)
//    imshow("VH lines cluster", img);
//    c = (char) waitKey(1);
//    c='m';
//    while(c!='q' && c!=27)
    {
        imshow("VH lines cluster", img);
        waitKey(1);
    }
//#endif
    return;
}

void RackDetect::getIntersections(vector<Vec4i> &vl, vector<Vec4i> &hl, vector<Vec2i> &points)
{
    for(int i=0; i<vl.size(); i++)
    {
        int y1 = vl[i](0), y2 = vl[i](2);
        int x1 = vl[i](1), x2 = vl[i](3);

        for(int j=0; j<hl.size(); j++)
        {
            int y3 = hl[j](0), y4 = hl[j](2);
            int x3 = hl[j](1), x4 = hl[j](3);

            int px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
            int py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
            Vec2i v(px,py);
            points.push_back(v);
        }
    }
    return;
}

bool RackDetect::point2Dto3D(Vec2i pixel, PointXYZ &point3d)
{
    int width = this->ref_cloud->width, height = this->ref_cloud->height;
    int row = pixel[0], col = pixel[1];

//    cout << "Kernel: " ;
    for(int kernel=0; kernel<25; kernel++)
    {
        bool is_nan = true;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;
//        cout << kernel << " ";
        for(int n=row-kernel; n<=row+kernel; n++)
        {
            for(int m=col-kernel; m<=col+kernel; m++)
            {
                if(n >=0 && n < height && m >= 0 && m < width)
                {
                    PointXYZRGBA& pt = this->ref_cloud->at(n*width+m);
                    if(!isnan(pt.x))
                    {
                        sum_x += pt.x;
                        sum_y += pt.y;
                        sum_z += pt.z;
                        count++;
                        is_nan = false;
                    }
                }
            }
        }
        if(!is_nan)
        {
            point3d.x = sum_x/count;
            point3d.y = sum_y/count;
            point3d.z = sum_z/count;
            return true;
        }
        else
            continue;
//            return false;// could not get any valid point within a kernel of size 10*10
    }
//    cout << endl;
    return false;

}

void RackDetect::clusterPoints(vector<Vec3d> &in_pts, vector<Vec3d> &out_pts)
{
    vector<int> flag(in_pts.size(), 0);
    int n = 0;

    // create cluster of the points which are within a distance of 7.5cm
    for(int i=0; i<in_pts.size(); i++)
    {
        if(flag[i]==0)
        {
            n++;
            Vec3d pt1 = in_pts[i];
            double sum_x = pt1(0);
            double sum_y = pt1(1);
            double sum_z = pt1(2);

            int cnt=1;
            for(int j=i+1; j<in_pts.size(); j++)
            {
                Vec3d pt2 = in_pts[j];
                double d=sqrt(pow(pt1(0)-pt2(0),2)+pow(pt1(1)-pt2(1),2)+pow(pt1(2)-pt2(2),2));
                if(d<0.075)//cluster as one point if distance between them is less than 7.5cm
                {
                    sum_x += pt2(0);
                    sum_y += pt2(1);
                    sum_z += pt2(2);
                    cnt++;
                    flag[j] = 1;
                }
            }
            Vec3d v(sum_x/cnt, sum_y/cnt, sum_z/cnt);
            out_pts.push_back(v);
#if(DEBUG_MODE)
            cout << "Final " << n << ": [" << v(0) << " " << v(1) << " " << v(2) << "]" << endl;
#endif
        }
    }
}

void RackDetect::rearrangeClusters(vector<Vec3d> &points)
{
    vector<Vec3d> left_pts;
    vector<Vec3d> right_pts;
    // splits the points into left and right vertical points
    // Y_LIMIT_MIN<y<Y_LIMIT_MAX to consider block of middle three rows
    for(int i=0; i<points.size(); i++)
    {
        if(points[i](0) > 0)// && points[i](1) > Y_LIMIT_MIN && points[i](1) < Y_LIMIT_MAX)// right line point
        {
            right_pts.push_back(points[i]);
        }
        else if(points[i](0))// < 0 && points[i](1) > Y_LIMIT_MIN && points[i](1) < Y_LIMIT_MAX)// left line point
        {
            left_pts.push_back(points[i]);
        }
    }
#if(DEBUG_MODE)
    cout << "No. of points on right line: " << right_pts.size() << endl;
    cout << "No. of points on left line: " << left_pts.size() << endl;
#endif

    // rearrange right line points in ascending order of y-dir of kinect
    for(int i=0; i<right_pts.size(); i++)
    {
        for(int j=i+1; j<right_pts.size(); j++)
        {
            if(right_pts[i](1) > right_pts[j](1))
            {
                Vec3d tmp = right_pts[i];
                right_pts[i] = right_pts[j];
                right_pts[j] = tmp;
            }
        }
    }
    // rearrange left line points in ascending order of y-dir of kinect
    for(int i=0; i<left_pts.size(); i++)
    {
        for(int j=i+1; j<left_pts.size(); j++)
        {
            if(left_pts[i](1) > left_pts[j](1))
            {
                Vec3d tmp = left_pts[i];
                left_pts[i] = left_pts[j];
                left_pts[j] = tmp;
            }
        }
    }

    if(right_pts.size() == 3 && left_pts.size() == 3)
    {
        points.resize(6);
        for(int i=0; i<points.size(); i+=2)
        {
            points[i] = left_pts[i/2];
            points[i](0) += V_LINES_REDUCTION;// bring left vertical points little bit to right
        }
        for(int i=1; i<points.size(); i+=2)
        {
            points[i] = right_pts[(i-1)/2];
            points[i](0) -= V_LINES_REDUCTION;// bring right vertical points little bit to left
        }
    }
    else
        points.resize(0);
    return;
}

void RackDetect::formRackCorners(vector<Vec3d> &points)
{
    Eigen::Vector3d v;

    // Find and store the 4 corners of rack
    v(0) = points[0](0)-points[2](0);
    v(1) = points[0](1)-points[2](1);
    v(2) = points[0](2)-points[2](2);
    v.normalize();
    v *= HEIGHT_EXT;
    Eigen::Vector4d tl;
    tl(0) = v(0) + points[0](0);
    tl(1) = v(1) + points[0](1);
    tl(2) = v(2) + points[0](2);
    tl(3) = 1.0;

    Eigen::Vector4d bl;
    bl(0) = points[4](0) - v(0);
    bl(1) = points[4](1) - v(1);
    bl(2) = points[4](2) - v(2);
    bl(3) = 1.0;

    v(0) = points[1](0)-points[3](0);
    v(1) = points[1](1)-points[3](1);
    v(2) = points[1](2)-points[3](2);
    v.normalize();
    v *= HEIGHT_EXT;
    Eigen::Vector4d tr;
    tr(0) = v(0) + points[1](0);
    tr(1) = v(1) + points[1](1);
    tr(2) = v(2) + points[1](2);
    tr(3) = 1.0;

    Eigen::Vector4d br;
    br(0) = points[5](0) - v(0);
    br(1) = points[5](1) - v(1);
    br(2) = points[5](2) - v(2);
    br(3) = 1.0;

    this->rack_corners[0] = tl;
    this->rack_corners[3] = tr;
    this->rack_corners[16] = bl;
    this->rack_corners[19] = br;

    // store the remaining vertical edge line corners
    Eigen::Vector4d corner;
    corner = Eigen::Vector4d (points[0](0),points[0](1), points[0](2), 1.0);
    this->rack_corners[4] = corner;
    corner = Eigen::Vector4d (points[2](0),points[2](1), points[2](2), 1.0);
    this->rack_corners[8] = corner;
    corner = Eigen::Vector4d (points[4](0),points[4](1), points[4](2), 1.0);
    this->rack_corners[12] = corner;

    corner = Eigen::Vector4d (points[1](0),points[1](1), points[1](2), 1.0);
    this->rack_corners[7] = corner;
    corner = Eigen::Vector4d (points[3](0),points[3](1), points[3](2), 1.0);
    this->rack_corners[11] = corner;
    corner = Eigen::Vector4d (points[5](0),points[5](1), points[5](2), 1.0);
    this->rack_corners[15] = corner;

    // Store corners for the middle two vertical lines
    for(int i=0; i<H_LINES; i++)
    {
        v(0) = this->rack_corners[i*4+3](0) - this->rack_corners[i*4](0);
        v(1) = this->rack_corners[i*4+3](1) - this->rack_corners[i*4](1);
        v(2) = this->rack_corners[i*4+3](2) - this->rack_corners[i*4](2);
        v.normalize();

        this->rack_corners[i*4+1](0) = v(0)*WIDTH_EXT + this->rack_corners[i*4](0);
        this->rack_corners[i*4+1](1) = v(1)*WIDTH_EXT + this->rack_corners[i*4](1);
        this->rack_corners[i*4+1](2) = v(2)*WIDTH_EXT + this->rack_corners[i*4](2);

        this->rack_corners[i*4+2](0) = v(0)*(WIDTH_EXT+WIDTH_MID) + this->rack_corners[i*4](0);
        this->rack_corners[i*4+2](1) = v(1)*(WIDTH_EXT+WIDTH_MID) + this->rack_corners[i*4](1);
        this->rack_corners[i*4+2](2) = v(2)*(WIDTH_EXT+WIDTH_MID) + this->rack_corners[i*4](2);
    }


    return;
}

void RackDetect::rangeCrop(PointCloud<PointXYZRGBA>::Ptr &in_cloud, PointCloud<PointXYZRGBA>::Ptr &out_cloud,
                           bool to_dense)
{
    vector<int> range_max(3), range_min(3);
    range_max[0] = 2.0;
    range_max[1] = 2.0;
    range_max[2] = 2.0;

    range_min[0] = -2.0;
    range_min[1] = -2.0;
    range_min[2] = -2.0;

    if(to_dense)
    {
        cout << "Cropping to dense" << endl;
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        vector<int> indices;
        for(int i=0; i<in_cloud->points.size(); i++)
        {
            PointXYZRGBA pt = in_cloud->points[i];
//            cout << i << ": " << pt << endl;
//            getchar();
            if(!isnan(pt.x))
            {
                if(range_min[0] < pt.x && pt.x < range_max[0] &&
                        range_min[1] < pt.y && pt.y < range_max[1] &&
                        range_min[2] < pt.z && pt.z < range_max[2])
                {
//                    cout << i << ": " << pt << endl;
//                    inliers->indices.push_back(i);
                    indices.push_back(i);

                }
            }
        }
//        // Extract the inliers
//        extract.setInputCloud (in_cloud);
//        extract.setIndices (inliers);
//        extract.setNegative (false);
//        extract.filter (*out_cloud);
        out_cloud->is_dense = true;
//        out_cloud->resize(inliers->indices.size());
        out_cloud->resize(indices.size());
        for(int i=0; i<indices.size();i++)
//            if(in_cloud->)
            out_cloud->points[i] = in_cloud->points[indices[i]];
        cout << "Dense cropping obtained" << endl;
    }
    else
    {
        out_cloud->width = in_cloud->width;
        out_cloud->height = in_cloud->height;
        out_cloud->is_dense = false;
        out_cloud->points.resize(out_cloud->width*out_cloud->height);

        const float bad_point = std::numeric_limits<float>::quiet_NaN();

        cout << "Range cropping" << endl;
    //    Get the indices of dense points within the range space
        for(int i=0; i<in_cloud->height; i++)
            for(int j=0; j<in_cloud->width; j++)
            {
                PointXYZRGBA pt = in_cloud->at(j,i);

                if(!isnan(pt.x))
                {
                    if(range_min[0] < pt.x && pt.x < range_max[0] &&
                            range_min[1] < pt.y && pt.y < range_max[1] &&
                            range_min[2] < pt.z && pt.z < range_max[2])
                        out_cloud->at(j,i) = pt;
                    else
                    {
                        out_cloud->at(j,i).x = bad_point;
                        out_cloud->at(j,i).y = bad_point;
                        out_cloud->at(j,i).z = bad_point;
                        out_cloud->at(j,i).rgba = pt.rgba;
                    }
                }
                else
                {
                    out_cloud->at(j,i) = pt;
                }
            }
    }

    return;
}


void RackDetect::getICPTransformation(PointCloud<PointXYZRGBA>::Ptr &ref, PointCloud<PointXYZRGBA>::Ptr &src)
{
    IterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> icp;

    icp.setInputSource(ref);
    icp.setInputTarget(src);
    PointCloud<PointXYZRGBA> final;
    icp.align(final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

    Eigen::Matrix<float,4,4> trn = icp.getFinalTransformation();// this is the transformation matrix from ref cloud to src cloud
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            this->transform(i,j) = trn(i,j);
    cout << endl << this->transform << endl;


    return;
}


void RackDetect::racktoBinCorners(vector<Eigen::Vector4d> &rk_crns, vector< vector<Eigen::Vector4d> > &bin_crns,
                                  vector<Eigen::Vector4d> &bin_cntd)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            bin_crns[i*3+j][0] = rk_crns[i*4+j];
            bin_crns[i*3+j][1] = rk_crns[i*4+1+j];
            bin_crns[i*3+j][2] = rk_crns[(i+1)*4+j];
            bin_crns[i*3+j][3] = rk_crns[(i+1)*4+1+j];

            bin_cntd[i*3+j] = (bin_crns[i*3+j][0] + bin_crns[i*3+j][1] +
                                          bin_crns[i*3+j][2] + bin_crns[i*3+j][3])/4;
        }
    }
}

void RackDetect::getBinPtsTransform(vector<Eigen::Vector4d> &vt)
{
    vector<Eigen::Vector4d> tf_v(vt.size());
    for(int i = 0; i < vt.size(); i++)
    {
        tf_v[i] = this->transform*vt[i];
    }
    return;
}

void RackDetect::printBinCorners()
{
    cout << "Corners of the bins" << endl;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            cout << "Bin " << i*3+j << ":\n "
                 << tf_bin_corners[i*3+j][0](0) << " " << tf_bin_corners[i*3+j][0](1) << " " << tf_bin_corners[i*3+j][0](2) << endl
                 << tf_bin_corners[i*3+j][1](0) << " " << tf_bin_corners[i*3+j][1](1) << " " << tf_bin_corners[i*3+j][1](2) << endl
                 << tf_bin_corners[i*3+j][2](0) << " " << tf_bin_corners[i*3+j][2](1) << " " << tf_bin_corners[i*3+j][2](2) << endl
                 << tf_bin_corners[i*3+j][3](0) << " " << tf_bin_corners[i*3+j][3](1) << " " << tf_bin_corners[i*3+j][3](2) << endl;
        }
    }
    return;
}

void RackDetect::printBinCentroids()
{
    cout << "Centroids for source cloud" << endl;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            cout << "Bin " << i*3+j << ":\n "
                 << tf_bin_centroids[i*3+j](0) << " " << tf_bin_centroids[i*3+j](1) << " " << tf_bin_centroids[i*3+j](2) << endl;
        }
    }
    return;
}

void RackDetect::addSphereMarkers(pcl::visualization::PCLVisualizer &visualizer,
                                  vector<Eigen::Vector4d> &corners,
                                  vector<Eigen::Vector4d> &centroids)
{
    float radii = .025;
    char id[10];
    string cor_id = "corner_";
    string cen_id = "centroid_";
    for(int i = 0; i<corners.size() ;i++)
    {
        pcl::PointXYZ point;
        point.x = corners[i](0);
        point.y = corners[i](1);
        point.z = corners[i](2);
        string str = cor_id;
        sprintf(id,"_%d",i);
        str.append(id);
        visualizer.addSphere<pcl::PointXYZ>(point, radii, 255.0, 0.0, 0.0, str);
    }
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
        {
            pcl::PointXYZ point;
            point.x = centroids[i*3+j](0);
            point.y = centroids[i*3+j](1);
            point.z = centroids[i*3+j](2);
            string str = cen_id;
            sprintf(id,"%d",i*3+j);
            str.append(id);
            visualizer.addSphere<pcl::PointXYZ>(point, radii, 0.0, 255.0, 0.0, str);
        }
    }
    return;
}

