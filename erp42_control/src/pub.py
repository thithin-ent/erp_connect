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
		self.speed = 1
		self.steering = 1
		self.mode = 0
		self.la = 'none'
		self.count = 0
		self.cv_image = None
		
		rospy.init_node('erp42_pub', anonymous=True)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.speedCallback)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.ctrl_Callback)
		rospy.Subscriber('/image_raw', Image, self.image_Callback)
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
		if self.mode == 1 :
			self.count += 1
			print(self.count)
			if Rspeed > 3:
				Rspeed = 3
			if self.count == 30:
				self.count = 0
				self.mode = 0
				
		elif self.mode == 2 and (self.la == 'Stop Signal\r' or self.la == 'Yellow Signal\r' or self.la == 'Straight Signal\r') : 
			while self.mode != 0 :
				Rspeed = 0
				steer = 0
				Spub.publish(0)
				Rpub.publish(0)
				print('mode 2 start')
				self.count += 1				
				if self.la == 'Left Signal\r': #or self.count == 100:
					self.count = 0
					self.mode = 0
				rospy.sleep(0.1)
		elif self.mode == 3 and (self.la == 'Stop Signal\r' or self.la == 'Yellow Signal\r' or self.la == 'Left Signal\r') :
			while self.mode != 0 :
				Spub.publish(0)
				Rpub.publish(0)
				print('mode 3 start')				
				if self.la == 'Straight Signal\r':
					self.mode = 0
				rospy.sleep(0.1)
		elif self.mode == 4:
				#image = cv2.imread("/home/kingofgps/test.jpeg", cv2.IMREAD_ANYCOLOR)
				if type(self.cv_image) != type(None) :
					mark = np.copy(self.cv_image)
					#print(type(mark))
					# white detect
					blue_threshold = 200
					green_threshold = 200
					red_threshold = 200
					bgr_threshold = [blue_threshold, green_threshold, red_threshold]
					thresholds = (self.cv_image[:,:,0] < bgr_threshold[0])  | (self.cv_image[:,:,1] < bgr_threshold[1]) | (self.cv_image[:,:,2] < bgr_threshold[2])
					mark[thresholds] = [0,0,0]
					gray = cv2.cvtColor(mark,cv2.COLOR_BGR2GRAY) 
					canny = cv2.Canny(gray, 5000, 1500, apertureSize = 5, L2gradient = True)
					lines = cv2.HoughLinesP(canny, 0.8, np.pi / 180, 90, minLineLength = 10, maxLineGap = 100)
					if type(lines) != type(None) :
					#	while self.count > 30 :
					#		self.count += 1
					#		Spub.publish(0)
					#		Rpub.publish(0)
					#		rospy.sleep(0.1)
					#	self.mode = 0
						for i in lines:
							if abs(i[0][1] - i[0][3]) < 20 and abs(i[0][0] - i[0][2]) > 280: #y pixel diff
								while self.count < 30 :
									self.count += 1
									print(self.count)
									Spub.publish(0)
									Rpub.publish(0)
									rospy.sleep(0.1)
								self.count = 0
								self.mode = 0
								cv2.line(self.cv_image, (i[0][0], i[0][1]), (i[0][2], i[0][3]), (255, 0, 0), 2)
								break
				
					cv2.imshow("test", self.cv_image)
					cv2.imshow("test2", canny)
					cv2.imshow("test3", mark)
					cv2.waitKey(1)
		elif self.mode == 5:
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
