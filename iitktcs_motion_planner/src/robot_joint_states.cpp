#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <pthread.h>
#include <std_msgs/Int16MultiArray.h>
#include <iitktcs_msgs_srvs/motor_input.h>

using namespace std;

// Specify the design id 1 or 2
#define USING_DESIGN 2
#define GRIPPER_REMOVED 1

// For gripper design 1
//// gripper in Rviz jt states
//#define GRIPPER_RETRACTED   -0.06
//#define GRIPPER_EXTENDED    0.0
//#define GRIPPER_CLOSE       -0.15
//#define GRIPPER_OPEN        0.0

//// gripper in real system
//#define G_SYS_RETRACTED 40
//#define G_SYS_FORWARD   90 //80
//#define G_SYS_H_CLOSE   115
//#define G_SYS_CLOSE     125//120

//// suction in Rviz jt states
//#define SUCTION_STRAIGHT    M_PI/2
//#define SUCTION_MID         M_PI/4
//#define SUCTION_BEND        0.0
//// suction in real system
//#define S_SYS_STRAIGHT      82
//#define S_SYS_STRA_TO_MID   60
//#define S_SYS_BEND_TO_MID   70
//#define S_SYS_BEND          52

//#define WAIT_TIME_SUCTION 1
//#define WAIT_TIME_GRIPPER_FORWARD 2
//#define WAIT_TIME_GRIPPER_RETRIEVE 2
//#define WAIT_TIME_GRIPPER_CLOSE 1

// For gripper design 2
// gripper in Rviz jt states
#define GRIPPER_RETRACTED   0.0
#define GRIPPER_EXTENDED    0.07
#define GRIPPER_CLOSE       0.075
#define GRIPPER_OPEN        0.07

// gripper in real system
#define G_SYS_RETRACTED 0
#define G_SYS_FORWARD   60
#define G_SYS_H_CLOSE   80
#define G_SYS_CLOSE     90

// suction in Rviz jt states
#define SUCTION_STRAIGHT    M_PI/2
#define SUCTION_MID         M_PI/4
#define SUCTION_BEND        0.0
// suction in real system
#define S_SYS_STRAIGHT      27
#define S_SYS_STRA_TO_MID   15
#define S_SYS_BEND_TO_MID   15
#define S_SYS_BEND          3

#define WAIT_TIME_SUCTION 1
#define WAIT_TIME_GRIPPER_FORWARD 4
#define WAIT_TIME_GRIPPER_RETRIEVE 2
#define WAIT_TIME_GRIPPER_CLOSE 2

// For gripper design 1
//int current_suction = 82, suction_req = 82, current_gripper = 40, gripper_req = 40;
// For gripper design 2
int current_suction = 27, suction_req = 27, current_gripper = 5, gripper_req = 5;
int wait_time = 1.0, gripper_movement_req = 0;

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_angles,
                         std::vector<double> &robot_joints)
{

    for (int i = 0; i < joint_angles->position.size(); i++)
    {
        if(joint_angles->name.at(i) == "shoulder_pan_joint")
            robot_joints[0] = joint_angles->position.at(i);
        if(joint_angles->name.at(i) == "shoulder_lift_joint")
            robot_joints[1] = joint_angles->position.at(i);
        if(joint_angles->name.at(i) == "elbow_joint")
            robot_joints[2] = joint_angles->position.at(i);
        if(joint_angles->name.at(i) == "wrist_1_joint")
            robot_joints[3] = joint_angles->position.at(i);
        if(joint_angles->name.at(i) == "wrist_2_joint")
            robot_joints[4] = joint_angles->position.at(i);
        if(joint_angles->name.at(i) == "wrist_3_joint")
            robot_joints[5] = joint_angles->position.at(i);
    }
    return;
}

void planningPositionCallback(const std_msgs::Int16MultiArray::ConstPtr &msg,
                              vector<double> &jt_position, vector<ros::Publisher> &jt_publishers,
                              ros::ServiceClient &client_hw_control, std_msgs::Int16 &suction_pos,
                              std_msgs::Int16 &gripper_pos)
{
    // msg->data[2] is flag for publishing to real gripper system
    // if msg->data[2] = 0 then publish on Rviz
    // if msg->data[2] = 1 then publish on real gripper system
    if(msg->data[2] == 0)
    {
#if(!GRIPPER_REMOVED)
        // Gripper data
        if(msg->data[0] == 0)
        {
            jt_position[7] = (double) GRIPPER_RETRACTED;
            jt_position[8] = (double) GRIPPER_OPEN;
            gripper_pos.data = (int) G_SYS_RETRACTED;
            gripper_req = (int) G_SYS_RETRACTED;

            if(gripper_req != current_gripper)
            {
                wait_time = (int) WAIT_TIME_GRIPPER_FORWARD;
                gripper_movement_req = 1;
            }
            else
                gripper_movement_req = 0;
            cout << "Gripper retracted" << endl;
        }
        else if(msg->data[0] == 1)
        {
            jt_position[7] = (double) GRIPPER_EXTENDED;
            jt_position[8] = (double) GRIPPER_OPEN;
            gripper_pos.data = (int) G_SYS_FORWARD;
            gripper_req = (int) G_SYS_FORWARD;

            if(gripper_req != current_gripper)
            {
                wait_time = (int) WAIT_TIME_GRIPPER_FORWARD;
                gripper_movement_req = 1;
            }
            else
                gripper_movement_req = 0;
            cout << "Gripper extended" << endl;
        }
        else if(msg->data[0] == 2)
        {
            jt_position[7] = (double) GRIPPER_EXTENDED;
            jt_position[8] = (double) GRIPPER_CLOSE;
            gripper_pos.data = (int) G_SYS_CLOSE;
            gripper_req = (int) G_SYS_CLOSE;

            if(gripper_req != current_gripper)
            {
                wait_time = (int) WAIT_TIME_GRIPPER_CLOSE;
                gripper_movement_req = 1;
            }
            else
                gripper_movement_req = 0;
            cout << "Gripper closed" << endl;
        }
#endif
        // Suction data
        if(msg->data[1] == 0)
        {
            jt_position[6] = (double) SUCTION_STRAIGHT;
            suction_pos.data = (int) S_SYS_STRAIGHT;
            suction_req = (int) S_SYS_STRAIGHT;
            cout << "current_suction: " << current_suction << endl;
            cout << "Suction req straight: " << suction_req << endl;

            if(gripper_movement_req == 0)
                wait_time = (int) WAIT_TIME_SUCTION;
        }
        else if(msg->data[1] == 1)
        {
            jt_position[6] = (double) SUCTION_MID;
            if(current_suction == S_SYS_BEND)
            {
                suction_pos.data = (int) S_SYS_BEND_TO_MID;
                suction_req = (int) S_SYS_BEND_TO_MID;
                cout << "current_suction: " << current_suction << endl;
                cout << "Suction req mid: " << suction_req << endl;
                cout << "Bend to Mid" << endl;
            }
            else if(current_suction == S_SYS_STRAIGHT)
            {
                suction_pos.data = (int) S_SYS_STRA_TO_MID;
                suction_req = (int) S_SYS_STRA_TO_MID;
                cout << "current_suction: " << current_suction << endl;
                cout << "Suction req mid: " << suction_req << endl;
                cout << "Straight to Mid" << endl;
            }
            else
            {
                suction_pos.data = current_suction;
                suction_req = current_suction;
                cout << "Retaining Suction Mid: " << current_suction << endl;
            }
            if(gripper_movement_req == 0)
                wait_time = (int) WAIT_TIME_SUCTION;
        }
        else if(msg->data[1] == 2)
        {
            jt_position[6] = (double) SUCTION_BEND;
            suction_pos.data = (int) S_SYS_BEND;
            suction_req = (int) S_SYS_BEND;
            cout << "current_suction: " << current_suction << endl;
            cout << "Suction req Bend: " << suction_req << endl;
            if(gripper_movement_req == 0)
                wait_time = (int) WAIT_TIME_SUCTION;
        }
        cout << endl;
    }
    // check the last index whether to publish data to execute gripper and suction position on real system
    else if(msg->data[2] == 1)
    {
#if(USING_DESIGN == 1)
        for(int i=0; i<2; i++)
        {
            jt_publishers[0].publish(gripper_pos);
            jt_publishers[1].publish(suction_pos);
        }
#elif(USING_DESIGN == 2)
        iitktcs_msgs_srvs::motor_input motor_input;

#if(!GRIPPER_REMOVED)
        motor_input.request.motor1 = gripper_pos.data;
#else
        motor_input.request.motor1 = 0.0;
#endif
        motor_input.request.motor2 = suction_pos.data;
        if(client_hw_control.call(motor_input))
            cout << "Gripper hardware control service complete" << endl;
        else
            cout << "Failed to call service " << client_hw_control.getService() << endl;
#endif
        current_suction = suction_req;
        current_gripper = gripper_req;
        if(gripper_movement_req == 1)
        {
            cout << "Wait time: " << wait_time << endl;
            sleep(wait_time);
        }
//        cout << "Executed on real system" << endl;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_js");
    ros::NodeHandle nh;

    // give your topic name here
    ros::Publisher sim_joint_publisher =
            nh.advertise<sensor_msgs::JointState>("/iitktcs/motion_planner/joint_states", 1);

    sensor_msgs::JointState jt_states;
    std_msgs::Int16 suction_position, gripper_position;
    gripper_position.data = (int) G_SYS_RETRACTED;
    suction_position.data = (int) S_SYS_STRAIGHT;
    current_suction =  (int) S_SYS_STRAIGHT;
    current_gripper = (int) G_SYS_RETRACTED;

#if(GRIPPER_REMOVED)
    int n_jts = 7;
#else
    int n_jts = 9;
#endif
    jt_states.name.resize(n_jts);
    jt_states.position.resize(n_jts); // total number of joints

    jt_states.name[0] = "shoulder_pan_joint";
    jt_states.name[1] = "shoulder_lift_joint";
    jt_states.name[2] = "elbow_joint";
    jt_states.name[3] = "wrist_1_joint";
    jt_states.name[4] = "wrist_2_joint";
    jt_states.name[5] = "wrist_3_joint";
    jt_states.name[6] = "suction_base_joint"; // provide your revolute joint name
#if(!GRIPPER_REMOVED)
    jt_states.name[7] = "gripper_body_joint";
    jt_states.name[8] = "gripper_left_finger_joint";
#endif

    bool flag_pub_gs_pos = false;

    vector<ros::Publisher> pub_joint_state_position(2);
#if(USING_DESIGN == 1)
    // For gripper design 1
    pub_joint_state_position[0] = nh.advertise<std_msgs::Int16>("/iitktcs/gripper_system/gripper_pos", 100);
    pub_joint_state_position[1] = nh.advertise<std_msgs::Int16>("/iitktcs/gripper_system/suction_pos", 100);

    for(int i=0; i<2; i++)
    {
        pub_joint_state_position[0].publish(gripper_position);
        pub_joint_state_position[1].publish(suction_position);
    }
#endif
    // For gripper design 2
    ros::ServiceClient client_gs2_hw_control = nh.serviceClient<iitktcs_msgs_srvs::motor_input>("iitktcs/gripper_system/send_motor_input");

    ros::Subscriber sub_joint_angle =
            nh.subscribe<sensor_msgs::JointState>
            ("/joint_states", 100,
             boost::bind(jointStatesCallback, _1, boost::ref(jt_states.position)));

    ros::Subscriber sub_planning_position =
            nh.subscribe<std_msgs::Int16MultiArray>
            ("/iitktcs/motion_planner/planning/suction_gripper_pos",100,
             boost::bind(planningPositionCallback, _1, boost::ref(jt_states.position),
                         boost::ref(pub_joint_state_position), boost::ref(client_gs2_hw_control), boost::ref(suction_position),
                         boost::ref(gripper_position)));

    jt_states.position[6] = (double) SUCTION_STRAIGHT; // default suction is straight
#if(!GRIPPER_REMOVED)
    jt_states.position[7] = (double) GRIPPER_RETRACTED; // default gripper is retracted
    jt_states.position[8] = (double) GRIPPER_OPEN; // left finger joint is fixed at 0 position
#endif

    while(ros::ok())
    {
        ros::spinOnce();

        sim_joint_publisher.publish(jt_states);
    }

    return 0;
}
