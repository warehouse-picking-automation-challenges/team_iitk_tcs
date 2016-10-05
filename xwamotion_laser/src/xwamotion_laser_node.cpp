#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include "xwam_motion/FwdBck.h"
#include "xwam_motion/UpDown.h"
#include <signal.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
#define V_LIMIT_SCISSOR 0.125
#define V_LIMIT_WHEELS 0.025
#define V_HIGH_WHEELS 0.25

#define HEIGHT_UPPER 0.615
#define HEIGHT_LOWER 0.285
#define HORIZONTAL_DIST_LIMIT 3.0//3.0
#define DELAY_TO_SCAN 2

#define WHEELS_SERVICE 1

geometry_msgs::Pose current_pose;
double current_height;
ros::Publisher pub;
double height, forward_dist;
bool called_once=false;

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;
    cout << "Exiting xwam motion" << endl;

    exit(1);
}

// xwam's laser gives 512 scan values between 0 to 180
// 2.84 scan lines per degree
// Have taken readings within approx. 4 degrees from center scan line
// scan line index 256 provides height of robot
// scan line index 0 provides forward distance of robot
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr scan_ptr, double &d_height, double &d_forward)
{
//    d_height = scan_ptr->ranges[256];
//    d_forward = scan_ptr->ranges[0];

    int d_idx = 256;
    int slide = 5;
    double avg_d=0;
    for(int i=d_idx-slide; i<d_idx+slide; i++)
    {
        avg_d += scan_ptr->ranges[i];
    }
    avg_d = avg_d/(slide*2);
    d_height = avg_d;

//    cout << "Index " << d_idx << ":\t" << avg_d << endl;

    d_idx = 0;
    slide = 10;
    avg_d=0;
    for(int i=d_idx; i<d_idx+slide; i++)
    {
        avg_d += scan_ptr->ranges[i];
    }
    avg_d = avg_d/slide;
    d_forward = avg_d;

//    cout << "Index " << d_idx << " :\t" << avg_d << endl;
    called_once = true;

    return;
}

bool scissorControlCallback(xwam_motion::UpDown::Request &req, xwam_motion::UpDown::Response &res)
{
    cout << "Reached service to move robot up down" << endl;

    // Velocity variable
    geometry_msgs::Twist cmd_vel;

    double z_data = req.height.data;
    double desired_height = height + z_data;
    double original = height;

    double error = desired_height-height;

    double time1 = ros::Time::now().toSec();

    double error_tol = 0.02;
    while(ros::ok() && fabs(error) > error_tol)// Continue till error is more than 2cm
    {
        double s = 0.25*(1-exp(fabs(error)));
        if(desired_height > height)
        {
            cmd_vel.linear.z = 0.25;
            pub.publish(cmd_vel);
        }
        else if(desired_height < height)
        {
            cmd_vel.linear.z = -0.25;
            pub.publish(cmd_vel);
        }
        sleep(1);

        ros::spinOnce();



        error = desired_height - height;
        if(fabs(error) < error_tol)
        {
            cout << "reached desired height" << endl;
            break;
        }
        else if(desired_height > height && height >= HEIGHT_UPPER)
        {
            cout << "reached maximum height" << endl;
            break;
        }
        else if(desired_height < height && height <= HEIGHT_LOWER)
        {
            cout << "reached minimum height" << endl;
            break;
        }
        else
        {
//            cout << "Original: " << original << ", Desired: " << desired_height << endl;
            cout << "D: " << desired_height << " C: " << height << " ";
            continue;
        }

        double time2 = ros::Time::now().toSec()-time1;
        cout << "time: " << time2;
        if(time2 > 15)
        {
            cout << "Time limit reached " << endl;
            break;
        }

    }
    ros::spinOnce();
    cout << "\nOriginal: " << original << ", Desired: " << desired_height << ", Current: " << height << endl;
    res.error.data = height - desired_height;

    return true;
}

#if(WHEELS_SERVICE)
bool wheelControlCallback(xwam_motion::FwdBck::Request &req, xwam_motion::FwdBck::Response &res)
#else
bool wheelControlCallback(double &d_move, double &b_error)
#endif
{
    cout << "Reached service to move robot fwd bwd" << endl;

    // Velocity variable
    geometry_msgs::Twist cmd_vel;
#if(WHEELS_SERVICE)
    double x_data = req.distance.data;
#else
    double x_data = d_move;
#endif

    double desired_distance = forward_dist - x_data;//forward_dist is global variable updated in laser callback
    double original = forward_dist;
    double error = desired_distance-forward_dist;
//    double kp = 0.2, ki = 0.0025, kd = 0.2;
//    double integral = 0.0, prev_errorX = error, derivative = 0;

    cout << "Desired: " << desired_distance << ", Current: " << forward_dist << endl;
    cout << "to move: " << x_data << endl;

    double error_tol = 0.02;
    while(ros::ok() && fabs(error) > error_tol)// Continue till error is more than 5mm
    {
        if(fabs(error) > HORIZONTAL_DIST_LIMIT)// if error > horizontal distance movement limit then something is wrong break out
        {
            cerr << "Error Distance to move: " << error << endl;
            return false;
        }
        if(desired_distance < forward_dist)
        {
//            integral = integral + (error*0.001);
//            derivative = (double)(error-prev_errorX)/(0.001);

//            cmd_vel.linear.x = -1*((kp * error) + (ki * integral) + (kd *derivative));
////            cmd_vel.linear.x=0.25;
//            prev_errorX = error;

//            if(fabs(error) > 0.075)
//                cmd_vel.linear.x = V_HIGH_WHEELS;
//            else
                cmd_vel.linear.x = V_LIMIT_WHEELS;
            pub.publish(cmd_vel);
        }
        else if(desired_distance > forward_dist)
        {
//            integral = integral + (error*0.001);
//            derivative = (double)(error-prev_errorX)/(0.001);

//            cmd_vel.linear.x = -1*((kp * error) + (ki * integral) + (kd *derivative));
////            cmd_vel.linear.x=-0.25;
//            prev_errorX = error;

//            if(fabs(error) > 0.075)
//                cmd_vel.linear.x = -V_HIGH_WHEELS;
//            else
                cmd_vel.linear.x = -V_LIMIT_WHEELS;
            pub.publish(cmd_vel);
        }

        sleep(DELAY_TO_SCAN);
        ros::spinOnce();

        error = desired_distance - forward_dist;
        if(fabs(error) < error_tol)
        {
            cout << "reached desired distance" << endl;
            break;
        }
        else
        {
//            cout << "original: " << original  << ", Desired: " << desired_distance << ", Current: " << forward_dist << endl;
            cout << "D: " << desired_distance << " C: " << forward_dist << " ";
            continue;
        }

    }

    cout << "original: " << original  << ", Desired: " << desired_distance << ", Current: " << forward_dist << endl;
#if(WHEELS_SERVICE)
    res.error.data = forward_dist - desired_distance;
#else
    b_error = forward_dist - desired_distance;
#endif

    return true;
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_callback_handler);

    ros::init(argc,argv,"xwamotion_laser");

    ros::NodeHandle nh;

    ros::ServiceServer service_fwdbck_motion = nh.advertiseService("/motion/fwdbck", wheelControlCallback);
    ros::ServiceServer service_updown_motion = nh.advertiseService("/motion/updown", scissorControlCallback);

    ros::Subscriber sub_laser = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(laserScanCallback,_1,boost::ref(height), boost::ref(forward_dist)));


    pub = nh.advertise<geometry_msgs::Twist>("/summit_xl_controller/command",1);

//    ros::MultiThreadedSpinner spinner(3);

//    spinner.spin();
    while(ros::ok())
        ros::spinOnce();

    return 0;
}
