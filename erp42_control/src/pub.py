#!/usr/bin/env python2

import serial
import time
import struct
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from autoware_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import TwistStamped
import sys
import os
import rospy
import message_filters
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class ERP42_Control():
	def __init__(self):
		self.bridge = CvBridge()
		self.speed = 0
		self.steering = 0
		self.mode = 4
		self.la = 'none'
		self.count = 0
		self.cv_image = None
		
		rospy.init_node('erp42_pub', anonymous=True)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.speedCallback)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.ctrl_Callback)
		rospy.Subscriber('/usb_cam/image_raw', Image, self.image_Callback)
		rospy.Subscriber('/detection/image_detector/objects', DetectedObjectArray, self.label_Callback)
			

	def pub_to_serial(self):
		self.mode_ch()
		self.ready_to_pub()
		rospy.sleep(0.1)

	def speedCallback(self, msg):
		self.speed = msg.twist.linear.x

	def ctrl_Callback(self, msg):
		self.steering = -msg.twist.angular.z
		
	def image_Callback(self, msg):
		self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		self.height = self.cv_image.shape[0]
		self.width = self.cv_image.shape[1]

	def label_Callback(self,msg):
		if msg.objects:
			self.la = msg.objects[0].label
		else :
			self.la = 'none'


	def mode_ch(self):
		if self.la == 'Child\r':			
			self.mode = 1
		elif self.la == 'Left\r':		
			self.mode = 2
		elif self.la == 'Straight\r':		
			self.mode = 3
		elif self.la == 'CrossWalk\r':
			self.mode = 4
		elif self.la == 'Parking\r':
			self.mode = 5
		

	def ready_to_pub(self):
		Spub = rospy.Publisher('/erp42/steer', Int16 , queue_size = 10)
		Rpub = rospy.Publisher('/erp42/speed', Int16 , queue_size = 10)
		Rspeed = self.speed
		steer = self.steering*180/3.141592
		if steer > 28 :
			steer = 28
		elif steer < -28 :
			steer = -28
		if self.mode == 1 :							# school zone or slow
			self.count += 1
			print(self.count)
			if Rspeed > 3:
				Rspeed = 3
			if self.count == 30:
				self.count = 0
				self.mode = 0
				
		elif self.mode == 2 and (self.la == 'Stop Signal\r' or self.la == 'Yellow Signal\r' or self.la == 'Straight Signal\r') :  # left turning 
			while self.mode != 0 :
				Rspeed = 0
				steer = 0
				Spub.publish(0)
				Rpub.publish(0)
				print('mode 2 start')
				self.count += 1				
				if self.la == 'Left Signal\r': #or self.count == 100:
					self.mode = 0
				rospy.sleep(0.1)
		elif self.mode == 3 and (self.la == 'Stop Signal\r' or self.la == 'Yellow Signal\r' or self.la == 'Left Signal\r') :	   # straight
			while self.mode != 0 :
				Spub.publish(0)
				Rpub.publish(0)
				print('mode 3 start')				
				if self.la == 'Straight Signal\r':
					self.mode = 0
				rospy.sleep(0.1)	
		elif self.mode == 4:								# stopline and crosswalk 
				if type(self.cv_image) != type(None) :
					hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
					h, s, v = cv2.split(hsv)			
					v = cv2.inRange(v, 200, 255)				# white dettect
					white = cv2.bitwise_and(hsv, hsv, mask = v)
					white = cv2.cvtColor(white, cv2.COLOR_HSV2BGR)
					vertices = np.array([[(50,self.height),(self.width/2-45, self.height/2+60), (self.width/2+45, self.height/2+60), (self.width-50,self.height)]], dtype=np.int32) # ROI
					mask = np.zeros_like(self.cv_image)
					cv2.fillPoly(mask, vertices, (255,255,255))
					mark = cv2.bitwise_and(mask, white)
					gray = cv2.cvtColor(mark,cv2.COLOR_BGR2GRAY) 
					canny = cv2.Canny(gray, 5000, 1500, apertureSize = 5, L2gradient = True)
					lines = cv2.HoughLinesP(canny, 0.8, np.pi / 180, 90, minLineLength = 10, maxLineGap = 100)
					if type(lines) != type(None) :
						for i in lines:
							#if abs(i[0][1] - i[0][3]) < 20 and abs(i[0][0] - i[0][2]) > 280: # pixel check
							cv2.line(self.cv_image, (i[0][0], i[0][1]), (i[0][2], i[0][3]), (255, 0, 0), 2)
					#			while self.count < 30 :
					#				self.count += 1
					#				print(self.count)
					#				Spub.publish(0)
					#				Rpub.publish(0)
					#				rospy.sleep(0.1)
					#			self.count = 0
					#			self.mode = 0
					#			break
				
					cv2.imshow("test", self.cv_image)
					cv2.imshow("test2", canny)
					cv2.imshow("test3", white)
					cv2.waitKey(1)
		elif self.mode == 5:								# parking
				print('mode 5')
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
