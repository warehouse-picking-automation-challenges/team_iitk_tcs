#include<iitktcs_msgs_srvs/pose.h>
#include<3d_pose_estimtion.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Pose_Estimate");
    EstimatePose est_pose;
    ros::spin();

    cout << "Hey header file is working" << endl;

    return 0;


}


