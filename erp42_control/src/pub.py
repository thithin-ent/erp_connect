#!/usr/bin/env python2

import serial
import time
import struct
from std_msgs.msg import Int16
#from 
import sys
import os
import rospy
import message_filters

class ERP42_Control():
	def __init__(self):
		self.speed = 0
		self.steering = 0
		self.mode = 1
		self.la = 'asdg'
		self.count = 0
		
		rospy.init_node('erp42_pub', anonymous=True)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.speedCallback)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.ctrl_Callback)
		rospy.Subscriber('/detection/image_detector/objects', DetectedObjectArray, self.label_read)

		self.
		self.pub = 
	def pub_to_serial(self):

		self.mode_ch()
		self.ready_to_pub()
		rospy.sleep(0.1)
		

	def speedCallback(self, msg):
		self.speed = msg.twist.linear.x

	def ctrl_Callback(self, msg):
		self.steering = -msg.twist.angular.z

	def label_read(self,msg):
		if msg.objects:
			self.la = msg.objects[0].label
		else :
			self.la = 'none'


	def mode_ch(self):
		if self.la == 'child\r':
			self.mode = 1
		elif self.la == 'Right\r':
			self.mode = 2
		

	def ready_to_pub(self):
		Spub = rospy.Publisher('/erp42/steer', Int16 , queue_size = 10)
		Rpub = rospy.Publisher('/erp42/speed', Int16 , queue_size = 10)
		Rspeed = self.speed
		steer = self.steering*180/3.141592
		if steer > 28 :
			steer = 28
		elif steer < -28 :
			steer = -28
		if self.mode == 1 :
			self.count += 1
			print(self.count)
			if Rspeed > 3:
				Rspeed = 3
			if self.count == 30:
				self.count = 0
				self.mode = 0
				
		elif self.mode == 2 && (self.la == 'red' || self.la == 'yellow' || self.la == 'green') : 
				Rspeed = 0
				if self.la == 'Right'
				self.mode = 0
				
		Spub.publish(int(steer))
		Rpub.publish(int(Rspeed))


		
if __name__ == '__main__':

	erp42_control = ERP42_Control()
	try:
		while not rospy.is_shutdown():
			erp42_control.pub_to_serial()

	except Exception as e:
		print(e)

