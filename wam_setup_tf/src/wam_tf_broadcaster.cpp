#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <kdl/frames.hpp>

#define PI 3.14159265

float cur_tilt_angle = 0;

void curTiltAngleCallback(const boost::shared_ptr<const std_msgs::Float64>& curTiltAngleMsg)
{
  cur_tilt_angle = curTiltAngleMsg->data;
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "wam_tf_publisher");
ros::NodeHandle n;
ros::Rate r(6);

tf::TransformBroadcaster j4_to_kinect_optical_frame;
tf::TransformBroadcaster kinect_int_frame;

tf::TransformBroadcaster kinect_fixed_frame;

ros::Subscriber angle_sub = n.subscribe("/cur_tilt_angle", 1, curTiltAngleCallback);

while(n.ok())
{
  ros::spinOnce();

  // Translation and Rotation should be given wrt parent(wam_link4) frame. Meaning obtain the required child frame by rotating
  // and translating the parent frame.
  // Euler ZYX, rotate wrt z, rotate wrt new y, rotate wrt new x. This is same as RPY rotate wrt x-axis, then with old y-axis,
  // then with old z-axis.
  // Here we obtain the wamj4_kinect_link(child frame) from the wam_link4(parent frame). wamj4_kinect_link is an intermediate link.
  // The kinect_base is rotated by x: anti-clockwise +90, y: no rotation(old y-axis), z: clockwise by -90(old z-axis)
  // Then the translation x: -0.045-0.125m, y: -0.14m, z: 0.0m
  // Rotation matrix R = |cos(-90) -sin(-90) 0|| cos(0)  0 sin(0)||1 0        0      |
  //                     |sin(-90)  cos(-90) 0|| 0       1 0     ||0 cos(90) -sin(90)|
  //                     |0         0        1||-sin(0)  0 cos(0)||0 sin(90)  cos(90)|

  double roll = 90*PI/180;
  double pitch = 0;
  double yaw = -90*PI/180;
  tf::Matrix3x3 R_j4_kinect;
  R_j4_kinect.setRPY(roll, pitch, yaw);

//  j4_to_kinect_optical_frame.sendTransform(
//  tf::StampedTransform(
//  tf::Transform(R_j4_kinect, tf::Vector3(-0.17, -0.14, 0.0)),
//  ros::Time::now(), "/wam_link4", "/wamj4_kinect_link"));




  // x-wam kinect transformation setup
  j4_to_kinect_optical_frame.sendTransform(
  tf::StampedTransform(
  tf::Transform(R_j4_kinect, tf::Vector3(-0.16, -0.15, 0.01)),
  ros::Time::now(), "/wam_link4", "/wamj4_kinect_link"));
//  float angle_radians = -4*PI/180;
  float angle_radians = 0*PI/180;
  float angle_radians_z = -1*PI/180;

  tf::Matrix3x3 kinect_tilt_mat;
  kinect_tilt_mat.setRPY(0.0, angle_radians, angle_radians_z);

  kinect_int_frame.sendTransform(
  tf::StampedTransform(
  tf::Transform(kinect_tilt_mat, tf::Vector3(-0, -0, 0.0)),
  ros::Time::now(), "/wamj4_kinect_link", "/camera_link"));

//  tf::Matrix3x3 wam_base_kinect_fixed(0.997489, -0.0230012, 0.0669863,
//                                      -0.0675617, -0.0252218, 0.997396,
//                                      -0.0212518, -0.999417, -0.0267125);


  // g-wam kinect transformation setup
//  j4_to_kinect_optical_frame.sendTransform(
//  tf::StampedTransform(
//  tf::Transform(R_j4_kinect, tf::Vector3(-0.16, -0.1, 0.01)),
//  ros::Time::now(), "/wam_link4", "/wamj4_kinect_link"));
//  float angle_radians = 1*PI/180; // stow task
//  float angle_radians_z = -1*PI/180;


//  tf::Matrix3x3 kinect_tilt_mat;
//  kinect_tilt_mat.setRPY(0.0, angle_radians, angle_radians_z);

//  kinect_int_frame.sendTransform(
//  tf::StampedTransform(
//  tf::Transform(kinect_tilt_mat, tf::Vector3(-0, 0.015, 0.0)),
//  ros::Time::now(), "/wamj4_kinect_link", "/camera_link"));






  tf::Matrix3x3 wam_base_kinect_fixed(0.997489, -0.0230012, 0.0669863,
                                      -0.0675617, -0.0252218, 0.997396,
                                      -0.0212518, -0.999417, -0.0267125);

//  wam_base_kinect_fixed.setRPY(-90*PI/180,0.0,-90*PI/180);
  kinect_fixed_frame.sendTransform(
  tf::StampedTransform(
  tf::Transform(wam_base_kinect_fixed, tf::Vector3(-0.01, -0.38, 0.23)),
  ros::Time::now(), "/wam_base_link", "/fixed_kinect"));

  r.sleep();
  }
}
