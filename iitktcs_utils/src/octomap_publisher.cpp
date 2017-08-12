#include<octomap_publisher.h>



int main(int argc, char **argv)
{
    ros::init(argc,argv,"Octomap_Publisher");
    PublishOctomap pub_oct;

    cout << "Sharath properitary algorithm" << endl;
    cout << "Hey !!!!!!! Octomap is going to rock" << endl;
    ros::spin();

    return 0;

}
