#ifndef MOVEIT_INTERFACE_H
#define MOVEIT_INTERFACE_H

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

//moveit::planning_interface::MoveGroup arm_group("g_wam_arm");

//ros::Publisher pub_display_trajectory = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);

void displayTrajectory(std::vector<double> &jt_angles, moveit::planning_interface::MoveGroup &group, ros::Publisher &pub)
{
//    group.setPlanningTime(10.0);

    group.setJointValueTarget(jt_angles);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
//    group.execute(my_plan);
    group.move();

//    moveit_msgs::DisplayTrajectory display_trajectory;
//    display_trajectory.trajectory_start = my_plan.start_state_;
//    display_trajectory.trajectory.push_back(my_plan.trajectory_);

//    pub_display_trajectory.publish(display_trajectory);

    return;

}

#endif // MOVEIT_INTERFACE_H
