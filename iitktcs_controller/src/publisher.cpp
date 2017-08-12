#include <ros/ros.h>
#include <typeinfo>
#include <iitktcs_msgs_srvs/publish_on_topic.h>
#include <iitktcs_msgs_srvs/gripper_controller.h>
#include <std_msgs/Int16MultiArray.h>

using namespace std;

class PUBLISHER {
private:
protected:
public:
    ros::NodeHandle nh;
    bool publish_on_topic_suction_valve_server(iitktcs_msgs_srvs::publish_on_topicRequest &req, iitktcs_msgs_srvs::publish_on_topicResponse &res);
    bool publish_on_topic_gripper_server(iitktcs_msgs_srvs::gripper_controllerRequest &req, iitktcs_msgs_srvs::gripper_controllerResponse &res);
    ros::ServiceServer publish_topic_suction_valve, publish_topic_gripper;
    ros::Publisher publish_on_topic_suction = nh.advertise<std_msgs::Int16>("/iitktcs/arduino/vacuum_ctrl",1);
    ros::Publisher publish_on_topic_valve = nh.advertise<std_msgs::Int16>("/iitktcs/valve",1);
    ros::Publisher publish_on_topic_gripper = nh.advertise<std_msgs::Int16MultiArray>("/iitktcs/motion_planner/planning/suction_gripper_pos",1);
    PUBLISHER();
    ~PUBLISHER();
};

bool PUBLISHER::publish_on_topic_suction_valve_server(iitktcs_msgs_srvs::publish_on_topicRequest &req, iitktcs_msgs_srvs::publish_on_topicResponse &res)
{
    string topic_for = req.topic_name.data;
    if(topic_for == "vacuum")
        publish_on_topic_suction.publish(req.data_to_publish);
    else if(topic_for == "valve")
        publish_on_topic_valve.publish(req.data_to_publish);
    res.success.data = true;
    return true;
}

bool PUBLISHER::publish_on_topic_gripper_server(iitktcs_msgs_srvs::gripper_controllerRequest &req, iitktcs_msgs_srvs::gripper_controllerResponse &res)
{
    publish_on_topic_gripper.publish(req.control_data);
    res.success.data = true;
    return true;
}

PUBLISHER::PUBLISHER()
{
    PUBLISHER::publish_topic_suction_valve = nh.advertiseService("/publish_topic_suction_valve",&PUBLISHER::publish_on_topic_suction_valve_server,this);
    PUBLISHER::publish_topic_gripper = nh.advertiseService("/publish_topic_gripper",&PUBLISHER::publish_on_topic_gripper_server,this);
}

PUBLISHER::~PUBLISHER()
{

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"publisher");
    PUBLISHER *publisher = new PUBLISHER();
    ros::spin();
    return 0;
}
