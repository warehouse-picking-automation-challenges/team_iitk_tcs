#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PI 3.14159265
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrated_tf_publisher");
    ros::NodeHandle n;
//    ros::Rate r(100);

    tf::TransformBroadcaster T_cl_ee;


//    // ensenso rotated 45 with mount at eef for gripper design 1
//    tf::Matrix3x3 R_ee_cl(0.0903809,  0.190623, 0.977494,
//                          -0.690086, -0.695695, 0.199475,
//                           0.718062, -0.692584, 0.0686686);

    // in iit
//    sensor to rbt
//     0.0711087   0.219396   0.973041  0.0396188
//     -0.997351 0.00064995  0.0727387  0.0114124
//     0.0153261  -0.975636   0.218861   0.160689
//             0          0          0          1
//    rbt to sensor
//     0.0711087  -0.997351  0.0153261 0.00610222
//      0.219396 0.00064994  -0.975636   0.148074
//      0.973041  0.0727386   0.218861 -0.0745494
//             0          0          0          1
    // in japan
//    sensor to rbt
//     0.0756697   0.217902   0.973033  0.0373014
//     -0.997109 0.00977505   0.075353 0.00166302
//    0.00690796  -0.975922   0.218012    0.15264
//             0          0          0          1
//    rbt to sensor
//      0.0756697   -0.997109  0.00690768 -0.00221878
//       0.217902  0.00977512   -0.975922     0.14082
//       0.973033   0.0753528    0.218012  -0.0696987
//              0           0           0           1
    // ensenso 0 degrees without mount at eef for gripper design 2
    tf::Matrix3x3 R_ee_cl(0.0756697,   0.217902,   0.973033,
                          -0.997109, 0.00977505,   0.075353,
                         0.00690796,  -0.975922,   0.218012);

    while(n.ok())
    {
        // ensenso rotated 45 with mount at eef for gripper design 1
//        T_cl_ee.sendTransform(
//                    tf::StampedTransform(
//                        tf::Transform(R_ee_cl, tf::Vector3 (0.0432301, 0.115773, 0.122649)),
//                        ros::Time::now(), "/ee_link", "/camera_link"));

        // ensenso 0 degrees without mount at eef for gripper design 2
        T_cl_ee.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(R_ee_cl, tf::Vector3 (0.0373014, 0.00166302, 0.15264)),
                        ros::Time::now(), "/ee_link", "/camera_link"));

//        r.sleep();
        usleep(1);
    }



    return 0;

}

