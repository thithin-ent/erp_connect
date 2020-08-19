#!/usr/bin/env python2

import time
import struct
from std_msgs.msg import Int16
from autoware_msgs.msg import VehicleStatus
import sys
import os
import rospy
import message_filters
import numpy as np

class ERP_Status():
	def __init__(self):
		self.Rspeed = 2
		self.steering = 1
		
		rospy.init_node('status', anonymous=True)
		rospy.Subscriber('/speed_status', Int16, self.speedCallback)
		rospy.Subscriber('/steering_status', Int16, self.ctrl_Callback)
		self.pub = rospy.Publisher('/status',VehicleStatus, queue_size=1)


	def set_to_status(self):
		self.get_in_status()
		rospy.sleep(0.1)
			


	def speedCallback(self, msg):
		self.Rspeed = msg

	def ctrl_Callback(self, msg):
		self.steering = msg


	def get_in_status(self):
		vehicle = VehicleStatus()
		print(self.Rspeed)
		vehicle.speed = float(self.Rspeed)/10
		print(type(vehicle.speed))
		print(vehicle.speed)
		vehicle.angle = (self.steering*np.pi)/(71*180)
		self.pub.publish(vehicle)
			


if __name__== '__main__':
	erp_status = ERP_Status()
	print("111111111")
	try:
		while not rospy.is_shutdown():
			erp_status.set_to_status()
	except Exception as e:
		print(e)
	
