#include<iostream>
#include<ros/ros.h>
#include<fstream>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc,argv,"launch");
    ros::NodeHandle nh;


    vector<string> model_names;

    if(!nh.getParam("/ARC17_OBJECT_NAMES",model_names))
        ROS_ERROR("Unable to load parameter");

    cout << "fist model name" << model_names[0] << endl;

    ofstream file;

    stringstream ss1;
    ss1 << "/home/manish/new_ws/src/iitktcs_pose_estimation/launch/new_param.launch";
    file.open(ss1.str().c_str());
    file << "<?xml version=\"1.0\"?>" ;
    file << "\n\n";
    file << "<launch> \n";
    file << "\t" ;
    file << "<rosparam command=\"load\" file=\"$(find iitktcs_pose_estimation)/param/bin_corners.yaml\"  />";
    file << "\n";
    for(int i=0;i<atoi(argv[1]);i++)
    {
        stringstream ss;
        file << "\t" ;
        ss << "<rosparam command=\"load\" file=\"$(find iitktcs_pose_estimation)/param/"
              << model_names[i] << ".yaml" << "\" />";

        file << ss.str();
        file << "\n";


    }
    file << "</launch>";
    file.close();



}
