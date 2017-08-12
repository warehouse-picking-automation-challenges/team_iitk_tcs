#include <ros/ros.h>
#include <signal.h>
#include <iitktcs_motion_planner/robotarm.h>

using namespace std;
#define COMMENT_ARM_GROUP 1

void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"iitktcs_motion_planner");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    signal(SIGINT, signal_callback_handler);


#if(COMMENT_ARM_GROUP)
    RobotArm arm_control;
#endif

    ros::ServiceServer service_single_goal, service_suction_pose_goal, service_gripper_pose_goal;
    ros::ServiceServer service_fk, service_test_calibration, service_retreive_robot, service_valley_planning;

    // using class member function as call back function- giving error have to resolve later
    service_single_goal = node.advertiseService<iitktcs_msgs_srvs::UR5Goal::Request, iitktcs_msgs_srvs::UR5Goal::Response>
            ("/iitktcs/motion_planner/single_goal",
             boost::bind(&RobotArm::serviceSingleGoalCallback, &arm_control, _1, _2));

    service_suction_pose_goal =
            node.advertiseService<iitktcs_msgs_srvs::UR5PoseGoal::Request, iitktcs_msgs_srvs::UR5PoseGoal::Response>
            ("/iitktcs/motion_planner/suction_pose_goal",
             boost::bind(&RobotArm::serviceSuctionPoseGoalCallback, &arm_control, _1, _2));

//    service_gripper_pose_goal =
//            node.advertiseService<iitktcs_msgs_srvs::UR5PoseGoal::Request, iitktcs_msgs_srvs::UR5PoseGoal::Response>
//            ("/iitktcs/motion_planner/gripper_pose_goal",
//             boost::bind(&RobotArm::serviceGripperPoseGoalCallback, &arm_control, _1, _2));

    service_gripper_pose_goal =
            node.advertiseService<iitktcs_msgs_srvs::GripperPoseGoal::Request, iitktcs_msgs_srvs::GripperPoseGoal::Response>
            ("/iitktcs/motion_planner/gripper_pose_goal",
             boost::bind(&RobotArm::serviceGripperGeneralizedPlanCallback, &arm_control, _1, _2));

    service_fk =
            node.advertiseService<iitktcs_msgs_srvs::GetFK::Request, iitktcs_msgs_srvs::GetFK::Response>
            ("/iitktcs/motion_planner/forward_kinematics",
             boost::bind(&RobotArm::serviceGetFKCallback, &arm_control, _1, _2));

    service_retreive_robot =
            node.advertiseService<iitktcs_msgs_srvs::Retrieval::Request, iitktcs_msgs_srvs::Retrieval::Response>
            ("/iitktcs/motion_planner/retrieve_from_bin",
             boost::bind(&RobotArm::serviceRetrieveRobotCallback, &arm_control, _1, _2));

    service_test_calibration =
            node.advertiseService<iitktcs_msgs_srvs::TestCalibration::Request, iitktcs_msgs_srvs::TestCalibration::Response>
            ("/iitktcs/motion_planner/test_calibration",
             boost::bind(&RobotArm::serviceTestCalibrationCallback, &arm_control, _1, _2));

    ros::ServiceServer service_rviz_real_check =
            node.advertiseService<iitktcs_msgs_srvs::GripperRealRviz::Request, iitktcs_msgs_srvs::GripperRealRviz::Response>
            ("/iitktcs/gripper/rviz_real_check",
             boost::bind(&RobotArm::serviceGripperRvizRealCallback, &arm_control, _1, _2));

    service_valley_planning =
            node.advertiseService<iitktcs_msgs_srvs::UR5PoseGoal::Request, iitktcs_msgs_srvs::UR5PoseGoal::Response>
            ("/iitktcs/motion_planner/valley_planning",
             boost::bind(&RobotArm::serviceValleyPlanningCallback, &arm_control, _1, _2));

    while(ros::ok())
    {
        ros::spinOnce();
    }


    return 0;
}
