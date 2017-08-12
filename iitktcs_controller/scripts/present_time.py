#!/usr/bin/env python

import rospy

class present_time_in_millisec:
	def __init__(self):
		print " "
	def getTime(self):
		self.time_in_sec = rospy.get_rostime().secs
		self.time_in_nanosec = rospy.get_rostime().nsecs
		self.time_in_microsec = (self.time_in_sec * 1000000) + int(self.time_in_nanosec/1000)
		return self.time_in_microsec
