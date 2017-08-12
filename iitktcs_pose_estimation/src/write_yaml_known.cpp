#include<iostream>
#include<vector>
#include<fstream>
#include<string>
#include<sstream>
#include<ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>
#include <math.h>


using namespace std;
using namespace pcl;


void makeCylinder(int no_of_cylinder, vector<double> &param, string model_name)
{
    double PI = 3.14259,temp;

    //Parameters to change.
    float angle_gap = 7; //7 degree in radian.
    float pixel_gap = 0.004;

    float angle = ((angle_gap/180.0)*PI);
    int angle_multiplier = (360/angle_gap);
    int a=0,b=0;

//    for(int i=0;i<param.size();i++)
//        cout << param[i] << endl;
    //       vector<double> param;

    //       for (int i=0;i<no_of_cylinder;i++)
    //       {
    //           cout<< "Enter the radius of cylinder number " << i+1 << endl;
    //           cin>>temp;
    //           param.push_back(temp);
    //           b++;
    //           cout<< "Enter the height of cylinder number " << i+1 << endl;
    //           cin>>temp;
    //           param.push_back(temp);
    //           b++;
    //       }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 0;
    cloud->height = 1;

    int height_initial_value=0,height_final_value=0;

    for(int l=0;l<(param.size()/2);l++)
    {
        int radius_pixel_multiplier = param[2*l]/pixel_gap;
        int height_pixel_multiplier = param[2*l+1]/pixel_gap;
        height_final_value+=height_pixel_multiplier;
        for(int i=height_initial_value;i<=height_final_value;i+=1)
            for(int j=0;j<=radius_pixel_multiplier;j+=1)
                for(int theta=0;theta<=angle_multiplier;theta+=1)
                {
                    if(i==height_initial_value || i==height_final_value)
                    {
                        if(l==0 && i==height_initial_value || l==(param.size()/2-1) && i==height_final_value)
                        {
                            cloud->width+=1;
                            cloud->resize(cloud->width*cloud->height);
                            cloud->points[a].x=j*pixel_gap*cos(theta*angle);
                            cloud->points[a].y=j*pixel_gap*sin(theta*angle);
                            cloud->points[a].z=i*pixel_gap;
                            a+=1;
                        }
                        else if(l==(param.size()/2-1) && i==height_initial_value)
                        {
                            if(param[2*l]>param[2*l-2])
                            {
                                if((j*pixel_gap)>param[2*l-2])
                                {
                                    cloud->width+=1;
                                    cloud->resize(cloud->width*cloud->height);
                                    cloud->points[a].x=j*pixel_gap*cos(theta*angle);
                                    cloud->points[a].y=j*pixel_gap*sin(theta*angle);
                                    cloud->points[a].z=i*pixel_gap;
                                    a+=1;
                                }
                            }

                        }
                        else
                        {
                            if(param[2*l]>param[2*l+2])
                            {
                                if((j*pixel_gap)>param[2*l+2])
                                {
                                    cloud->width+=1;
                                    cloud->resize(cloud->width*cloud->height);
                                    cloud->points[a].x=j*pixel_gap*cos(theta*angle);
                                    cloud->points[a].y=j*pixel_gap*sin(theta*angle);
                                    cloud->points[a].z=i*pixel_gap;
                                    a+=1;
                                }
                            }
                            else
                            {
                                if((j*pixel_gap)>param[2*l-2])
                                {
                                    cloud->width+=1;
                                    cloud->resize(cloud->width*cloud->height);
                                    cloud->points[a].x=j*pixel_gap*cos(theta*angle);
                                    cloud->points[a].y=j*pixel_gap*sin(theta*angle);
                                    cloud->points[a].z=i*pixel_gap;
                                    a+=1;
                                }
                            }
                        }
                    }
                }
        for(int i=height_initial_value;i<=height_final_value;i+=1)
            for(int j=0;j<=radius_pixel_multiplier;j+=1)
                for(int theta=0;theta<=angle_multiplier;theta+=1)
                {
                    cloud->width+=1;
                    cloud->resize(cloud->width*cloud->height);
                    cloud->points[a].x=param[2*l]*cos(theta*angle);
                    cloud->points[a].y=param[2*l]*sin(theta*angle);
                    cloud->points[a].z=i*pixel_gap;
                    a+=1;
                }
        height_initial_value=height_final_value;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    pcl::PointCloud<pcl::PointXYZ> test;
    test.width = 1;
    test.height = 1;
    test.resize(test.height*test.width);
    test.points[0].x = centroid[0];
    test.points[0].y = centroid[1];
    test.points[0].z = centroid[2];

    pcl::visualization::PCLVisualizer vis("cloud");
    vis.addPointCloud(cloud);
    vis.addSphere(test.points[0],0.005,0.5,0.5,0.0,"sphere");
    vis.spin();
    vis.close();

    string object_name= "/home/manish/new_ws/src/iitktcs_pose_estimation/pcd1/";
    object_name+=model_name;
    object_name+=".pcd";

    pcl::io::savePCDFile(object_name,*cloud);


}


void makeCuboidal(vector<double> &dimensions,string model_name)
{
    double length = dimensions[0],breadth = dimensions[1],height = dimensions[2];
    double gap = 0.005;

    int l_multiplier = length/gap, b_multiplier = breadth/gap, h_multiplier = height/gap;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 0;
    cloud->height = 1;
    int a = 0;
    //    cout << int(length/gap) << " :: " << int(breadth/gap) << " :: " << int(height/gap) << endl;
    //    cout << length << " :: " << breadth << " :: " << height << endl;
    for (float i=0;i<=l_multiplier;i+=1)
        for (float j=0;j<=b_multiplier;j+=1)
            for(float k=0;k<=h_multiplier;k+=1)
                if( i==0 || j==0 || k==0 || i==l_multiplier || j==b_multiplier || k==h_multiplier )
                {
                    cloud->width += 1;
                    cloud->resize(cloud->width*cloud->height);
                    cloud->points[a].x = i*gap;
                    cloud->points[a].y = j*gap;
                    cloud->points[a].z = k*gap;
                    a++;
                }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    pcl::PointCloud<pcl::PointXYZ> test;
    test.width = 1;
    test.height = 1;
    test.resize(test.height*test.width);
    test.points[0].x = centroid[0];
    test.points[0].y = centroid[1];
    test.points[0].z = centroid[2];

    pcl::visualization::PCLVisualizer vis("cloud");
    vis.addPointCloud(cloud);
    vis.addSphere(test.points[0],0.005,0.5,0.5,0.0,"sphere");
    vis.spin();
    vis.close();

    string object_name= "/home/manish/new_ws/src/iitktcs_pose_estimation/pcd1/";
    object_name+=model_name;
    object_name+=".pcd";

    pcl::io::savePCDFile(object_name,*cloud);

}





int main(int argc, char **argv)
{
    ros::init(argc,argv,"writing_yaml");
    ros::NodeHandle nh;

    ofstream file;
    stringstream ss;

    vector<string> model_names;
    vector<int> current_model_ids;
    nh.getParam("/ARC17_OBJECT_NAMES",model_names);
    nh.getParam("/ARC17_COMPETITION_SET",current_model_ids);

    int size_model_id = current_model_ids.size();
    for(int i=0; i< 40 ; i++)
    {

        string object_name;
        object_name = model_names[i] ;
        cout << "Yaml file for " << object_name << endl;

        ss <<"/home/manish/new_ws/src/iitktcs_pose_estimation/param1/"  <<  object_name << ".yaml" ;
        file.open(ss.str().c_str());

        file << object_name << ":" << "\n";

        int data_normal1;
        int data_normal2;
        int data_normal3;

        cout  << "Enter the Nomral Data: " << endl;
        cin >> data_normal1;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> data_normal1;
        }
        cin >> data_normal2;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> data_normal2;
        }
        cin >> data_normal3;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> data_normal3;
        }


        file << " " << "normal_data: [" << data_normal1 << "," << data_normal2 << "," << data_normal3 << "]\n\n";

        cout << "Press 0 for cuboidal and 1 for cylinder object: " << endl;

        int geometry_type,object_type;
        cin >> geometry_type;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> geometry_type;
        }

        file << " " << "geometry_type: " << geometry_type << "\n\n";

        cout << "Press 0 for rigid and 1 for deformable: " << endl;
        cin >> object_type;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> object_type;
        }

        file << " " << "object_type: " << object_type << "\n\n";

        //    cout << "Press 0 for plane and 1 for cylinder fitting: " << endl;
        //    string primitive_type;
        //    cin >> primitive_type;
        file << " " << "primitive_type: " << geometry_type << "\n\n";

        cout << "Enter Super4Pcs parameter: " << endl;
        float overlap;
        cin >> overlap;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> overlap;
        }
        file << " " << "overlap: " << overlap  << "\n\n";


        cout << "Enter Dimensions of object" << endl;
        float dim1,dim2,dim3;
        cin >> dim1;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> dim1;
        }


        cin >> dim2 ;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> dim2;
        }

        cin>>dim3;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> dim3;
        }


        file << " " << "dimensions: [" << dim1 << "," << dim2 << "," << dim3 << "] \n\n";


        //    cout << "Enter Difficulty level: " << endl;
        //    int diff_level;
        //    cin >> diff_level;
        //    file <<" " << "difficulty: " << diff_level << "\n" ;

        cout << "Enter centroid offset " << endl;
        double centroid_offset;
        cin >> centroid_offset;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> centroid_offset;
        }
        file << " " << "offset: " << centroid_offset << "\n\n";


        //    cout << "Enter Fragile Nature" << endl;
        //    string fragility;
        //    cin >> fragility;
        //    file << " " << "fragility: " << fragility << "\n";


        cout << "Press 0 for light and 1 for heavy object : " << endl;
        int weight;
        cin >>weight;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> weight;
        }

        file << " " << "weight: " << weight << "\n\n";




        cout << "Press 0 for cubodial pcd 1 for cylinder" << endl;
        int input;
        cin >> input;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> input;
        }

        switch (input)
        {
        case 0:
        {
            cout << "making artificial pcd" << endl;
            vector<double> dimensions;
            dimensions.push_back(dim1);
            dimensions.push_back(dim2);
            dimensions.push_back(dim3);

            makeCuboidal(dimensions,object_name);
            break;
        }
        case 1:
        {
            cout << "making artificial pcd" << endl;
            int cylinder_input;
            cout << "Press 0 for one cylinder and 1 for multiple cylinder" << endl;
            cin >> cylinder_input;
            while(std::cin.fail())
            {
                std::cout << "Error" << std::endl;
                std::cin.clear();
                std::cin.ignore(256,'\n');
                std::cin >> cylinder_input;
            }
            if(cylinder_input==0)
            {
                vector<double > param;
                param.push_back(dim2/2);
                param.push_back(dim1);

                makeCylinder(1,param,object_name);

            }
            else
            {
                cout << "Enter no. of cylinders " << endl;
                int no_of_cylinder;
                cin >> no_of_cylinder;
                while(std::cin.fail())
                {
                    std::cout << "Error" << std::endl;
                    std::cin.clear();
                    std::cin.ignore(256,'\n');
                    std::cin >> no_of_cylinder;
                }
                vector<double> param;
                double b,a;
                for(int i=0; i< no_of_cylinder;i++)
                {
                    cout << "Enter radius and height for " << i << " cylinder" << endl;
                    cin >> b;
                    while(std::cin.fail())
                    {
                        std::cout << "Error" << std::endl;
                        std::cin.clear();
                        std::cin.ignore(256,'\n');
                        std::cin >> b;
                    }
                    cin >> a;
                    while(std::cin.fail())
                    {
                        std::cout << "Error" << std::endl;
                        std::cin.clear();
                        std::cin.ignore(256,'\n');
                        std::cin >> a;
                    }
                    param.push_back(b);
                    param.push_back(a);

                }
                makeCylinder(no_of_cylinder,param,object_name);

            }
            break;


        }

        }

        //    cout << file << endl;

        file.close();
        //    cout << "///////////////////////////////////////////////////////" << endl;
        cout << "*****************************************" << endl;
        ifstream f(ss.str().c_str());
        if (f.is_open())
            std::cout << f.rdbuf();

        //    cout << "///////////////////////////////////////////////////////" << endl;
        cout << "******************************************" << endl;
        int checker;
        cout << "Press 0 if file is write or 1 if wrong" << endl;
        cin >> checker;
        while(std::cin.fail())
        {
            std::cout << "Error" << std::endl;
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> checker;
        }
        if(checker == 1)
            i=i-1;


        ss.str("");

    }


    //    cout << "Enter Size criteria" << endl;
    //    string size;
    //    cin >>size;
    //    file << " " << "size: "  << size << "\n";

    return 0;

}

