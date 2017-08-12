#!/usr/bin/env python


import serial
import rospy
import struct
from std_msgs.msg import String

from std_msgs.msg import UInt16MultiArray
#from gripper.srv import motor_input
from iitktcs_msgs_srvs.srv import motor_input
from std_msgs.msg import Int64

class SerialIO:


	def __init__(self):

		self.serial_port=serial.Serial('/dev/ttyUSB0',9600)
	
		self.pubSerialData=rospy.Publisher('/serial_output',UInt16MultiArray,queue_size=1)
		self.head_flag=False

		self.data=[]
		
		self.motor_input=0
	
	def getSerialData(self):

		return self.serial_port.read()

	def publishData(self):

		new_value =struct.unpack('>1B',self.getSerialData())[0]

		if len(self.data)==8:

			serial_input=[(self.data[2] << 8) | self.data[1],
							(self.data[4] << 8) | self.data[3],
							 self.data[5],self.data[6]]

			pub_data=UInt16MultiArray()	
			pub_data.data=serial_input

			self.pubSerialData.publish(pub_data)

			self.head_flag=False
			self.data=[]

		if new_value==126 and self.head_flag==False:
			self.head_flag=True

		if self.head_flag:
		   self.data.append(new_value)



	def handleMotorInput(self,req):
		#print "Reached TCS proprietary gripper control service system"
		print "Sending Motor input %d , %d "%(int(req.motor1), int(req.motor2))

		self.serial_port.write(bytearray([126,int(req.motor1),int(req.motor2),253]))		

		return "Success"





if __name__=="__main__":

	rospy.init_node("SerialIO",anonymous=True)

	obj=SerialIO()
	
	head_flag=False

	data=[]

	s = rospy.Service('iitktcs/gripper_system/send_motor_input', motor_input, obj.handleMotorInput)
	
	while not rospy.is_shutdown():
		
		obj.publishData()

		


