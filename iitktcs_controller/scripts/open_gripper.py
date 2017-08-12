#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

class publisher():
	def __init__(self,text):
    		pub = rospy.Publisher('iitktcs/motion_planner/planning/suction_gripper_pos', Int16MultiArray, queue_size=2)
    		rate = rospy.Rate(10) # 10hz
    		count = 1
    		while not rospy.is_shutdown():
        		msg = Int16MultiArray()
			msg.data = text
        		pub.publish(msg)
        		rate.sleep()
			break
