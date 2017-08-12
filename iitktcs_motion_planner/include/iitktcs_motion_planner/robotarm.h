#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometric_shapes/shape_operations.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <rosserial_arduino/Adc.h>

#include <iitktcs_msgs_srvs/UR5Goal.h>
#include <iitktcs_msgs_srvs/UR5PoseGoal.h>
#include <iitktcs_msgs_srvs/GetFK.h>
#include <iitktcs_msgs_srvs/TestCalibration.h>
#include <iitktcs_msgs_srvs/Retrieval.h>
#include <iitktcs_msgs_srvs/GripperRealRviz.h>
#include <iitktcs_msgs_srvs/static_point_cloud.h>
#include <iitktcs_msgs_srvs/CheckClearProtectiveStop.h>
#include <iitktcs_msgs_srvs/GripperPoseGoal.h>
#include <iitktcs_msgs_srvs/gripper_suction_controller.h>

#define NO_JOINTS 6
#define SHOW_TRAJECTRY_POINTS 1
#define MARKERS_DURATION 600 // Markers duration is 30 seconds
#define VGS 1
#define NEW_COLLISION_MODELS 1
#define PICK_HEIGHT 0.05
#define ATTACH_OBJECT 0
#define ADD_COLLISION_OBJECT 0
#define USE_EXTRA_POINT_APPROACH_VECTOR_GRIPPER 1
#define PUBLISH_OCTOMAP 0
#define PLAN_GRIPPER_MESH_CUP 0
#define CHECK_FLOW_SENSOR 1
#define FLOW_SENSOR_TH 323

using namespace std;

class RobotArm
{
private:

    // planning group related
    moveit::planning_interface::MoveGroup *arm_group_ptr;
    moveit::planning_interface::MoveGroup *arm_group_straight, *arm_group_mid, *arm_group_bend, *arm_group_gripper;
    int current_group, selected_group;

    robot_state::RobotStatePtr kinematic_state;
    const robot_state::JointModelGroup *joint_model_group;
    string current_end_effector_link;

    // bin information
    vector<tf::Vector3> bin_corners;
    vector<tf::Vector3> bin_centroid;
    tf::Vector3 bin_normal, bin_axis, bin_top_vector;

    // gripper additional joints tf publisher related
    std_msgs::Int16MultiArray gripper_suction_sys_control;
    ros::Publisher pub_planning_position;
    bool gripper_closed;

    // planning related
    moveit::planning_interface::MoveGroup::Plan my_plan_cartesian_goal;
    moveit::planning_interface::MoveGroup::Plan my_plan_pose_joint_goal;
    moveit::planning_interface::MoveGroup::Plan my_plan_dummy;
    moveit::planning_interface::MoveItErrorCode *plan_result;

    double min_path_value, cartesian_path_fraction;
    double retrieve_pick_height, distance_along_normal;
    vector<double> approach_vector, current_joints;
    bool is_retrieval_plan, is_drop_plan, mesh_cup_gripper, got_protective_stop, protective_joint_target_moved;
    bool is_suction_plan = false;

    ros::ServiceClient client_clear_octomap, client_pub_octomap;
    iitktcs_msgs_srvs::static_point_cloud octomap_pub_clear;

    ros::ServiceClient client_checkclear_protective_stop, client_vacuum_cleaner_control;

    // picking object related
    int no_bins, bin_id, no_objects;
    int object_id;
    tf::Vector3 plan_axis, plan_normal;
    tf::Vector3 plan_major_axis, plan_minor_axis, object_model_centroid, object_hold_centroid;
    bool flag_plan_major_target_axis;
    tf::Vector3 object_centroid, object_normal;
    geometry_msgs::Pose object_attach_pose_world, object_attach_pose_suction, object_attach_pose_gripper;
    vector<string> model_names;
    vector<int> non_planning_item;
    ros::Publisher publisher_vacuum_control;
    ros::Subscriber subscriber_flow_sensor;
    double flow_sensor_adc;
    bool flag_get_flow_sensor_reading;

public:

    ros::NodeHandle nh;
    // Display markers related
    ros::Publisher pub_markerarray_traj_points, pub_markerarray_centroid, pub_markerarray_axis, pub_markerarray_normal;

    // planning scene and attached object in scene related
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    ros::Publisher planning_scene_diff_publisher;
    moveit_msgs::PlanningScene planning_scene;
    vector<string> touch_links;
    bool is_attached_object_present, added_object;

    ros::ServiceClient client_get_planning_scene;
    moveit_msgs::AllowedCollisionMatrix allowed_collision_matrix;

    moveit_msgs::CollisionObject *coll;
    moveit_msgs::CollisionObject *coll1;
    moveit_msgs::CollisionObject *coll2;
    moveit_msgs::CollisionObject *coll3;
    moveit_msgs::CollisionObject *coll4;
    moveit_msgs::CollisionObject *coll5;
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    vector<Eigen::Affine3d> reverse_points;

    vector<moveit_msgs::CollisionObject> collision_objects;
    vector<moveit_msgs::CollisionObject> collision_objects1;
    vector<moveit_msgs::CollisionObject> collision_objects2;

    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;

    vector<Eigen::Affine3d> retrace_waypoints_pose;

    RobotArm();
    ~RobotArm();

    vector<double> getCurrentJoints();
    void flowSensorCallback(const rosserial_arduino::AdcConstPtr &adc_ptr);
    bool checkJointsStateReached(vector<double> &desired_jts);
    bool sendCartesianGripperPoseGoal(geometry_msgs::Point &trg_centroid,
                               tf::Matrix3x3 &trg_orientation);
    bool genCartesianPlanPose(geometry_msgs::Point &trg_centroid, tf::Matrix3x3 &trg_orientation);
    bool genWayPointsPlan(std::vector<geometry_msgs::Pose> &waypoints);
    Eigen::Affine3d getFK(vector<double> joint_values = vector<double> (NO_JOINTS), bool fk_current = true);
    tf::Vector3 getEndpt(tf::Vector3 vector, tf::Vector3 &start_pt, double distance);
    void displayPlannerError(int &error_code);
    bool jointTargetGoal(vector<double> &goal);

    // service call back functions
    bool serviceSingleGoalCallback(iitktcs_msgs_srvs::UR5Goal::Request &req, iitktcs_msgs_srvs::UR5Goal::Response &res);
    bool serviceGripperPoseGoalCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req, iitktcs_msgs_srvs::UR5PoseGoal::Response &res);
    bool serviceGripperGeneralizedPlanCallback(iitktcs_msgs_srvs::GripperPoseGoal::Request &req,
                                               iitktcs_msgs_srvs::GripperPoseGoal::Response &res);
    bool serviceSuctionPoseGoalCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req, iitktcs_msgs_srvs::UR5PoseGoal::Response &res);
    bool serviceRetrieveRobotCallback(iitktcs_msgs_srvs::Retrieval::Request &req, iitktcs_msgs_srvs::Retrieval::Response &res);
    bool serviceGetFKCallback(iitktcs_msgs_srvs::GetFK::Request &req, iitktcs_msgs_srvs::GetFK::Response &res);
    bool serviceTestCalibrationCallback(iitktcs_msgs_srvs::TestCalibration::Request &req, iitktcs_msgs_srvs::TestCalibration::Response &res);
    bool serviceGripperRvizRealCallback(iitktcs_msgs_srvs::GripperRealRviz::Request &req, iitktcs_msgs_srvs::GripperRealRviz::Response &res);
    bool serviceValleyPlanningCallback(iitktcs_msgs_srvs::UR5PoseGoal::Request &req, iitktcs_msgs_srvs::UR5PoseGoal::Response &res);

    bool suctionMultipleAxisPlans(tf::Vector3 &axis1, tf::Vector3 &axis2, tf::Vector3 &normal,
                                  geometry_msgs::Point &centroid, vector<double> &current_jts, int &group_id);
    bool gripperMultipleAxisPlans(vector<tf::Vector3> &axis1_array, vector<tf::Vector3> &axis2_array, vector<tf::Vector3> &normal_array,
                                  vector<tf::Vector3> &centroid_array, vector<double> &current_jts, int &group_id);
    bool attemptSuctionPosePlan(tf::Vector3 axis, tf::Vector3 normal,
                                geometry_msgs::Point &centroid);
    bool attemptGripperPosePlan(tf::Vector3 axis, tf::Vector3 normal,
                                geometry_msgs::Point &centroid);

    double projection_distance(tf::Vector3 &normal_vector, tf::Vector3 &point_plane, tf::Vector3 &point);
    bool updateMinDistancePlan(vector<double> current_jts);
    geometry_msgs::Point alignAxes(vector<tf::Vector3> &valley_detect_axes, vector<tf::Vector3> object_axes,
                          tf::Vector3 valley_centroid, tf::Vector3 obj_hold_pt,
                          tf::Vector3 obj_model_centroid, bool invert_axes = false);

    // add marker functions
    void addSphereMarker(double *centroid, double *color, visualization_msgs::Marker &marker, std::string &frame_id, int id);
    void sphereMarker(vector<tf::Vector3> &points, ros::Publisher &marker_pub);
    void addArrowMarker(vector<tf::Vector3> &start_pt, vector<tf::Vector3> &end_pt, double *color,
                        visualization_msgs::Marker &marker, std::string &frame_id, int id);
    void arrowMarker(vector<tf::Vector3> &start_point, vector<tf::Vector3> &end_point,
                     ros::Publisher &marker_pub, double *line_color);
    void deleteMarkers(visualization_msgs::Marker &marker, std::string &frame_id, int id, string namesp, string marker_type);
    void plotAxisNormalCentroid(tf::Vector3 &axis, tf::Vector3 &normal, tf::Vector3 &centroid);
    void displayTrajectoryPoints(moveit::planning_interface::MoveGroup::Plan &plan, string type);

    void addPlates(int bin_target, vector<string> plate_ids);
    void addCollisionObject(geometry_msgs::Pose &object_pose, int object_id, string frame);
    void addPrimitiveCollisionObject(geometry_msgs::Pose &object_pose, int object_id, string frame);
    void removeCollisionObject(string frame);
    void attachCollisionObject();
    void detachCollisionObject();

    // allowed collision matrix related
    void publishBox();
    bool getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix &current_acm);
    void updatePlatesACM(moveit_msgs::AllowedCollisionMatrix &currentACM, vector<string> box1,
                         vector<string> box2);
    void updateTargetObjectACM(moveit_msgs::AllowedCollisionMatrix &currentACM, string object_name);

    bool executePlan(int &status_code, moveit::planning_interface::MoveGroup::Plan &plan,
                     bool wait_to_end = false);
    tf::StampedTransform getTransform(string source_frame, string target_frame);
    tf::Vector3 transformPointVector(tf::StampedTransform &transform, tf::Vector3 &point,
                                            bool is_vector_transform = false);
};

#endif // ROBOTARM_H
