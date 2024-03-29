#!/usr/bin/env python2

import serial
import time
import struct
from std_msgs.msg import UInt8, Int16, Float64
import sys
import os
import rospy
import message_filters

class ERP42_Serial():
	def __init__(self):
		self.ser = serial.Serial('/dev/ttyUSB0',115200)
		self.ser.bytesize = 8
		self.ser.parity = 'N'
		self.ser.stopbits = 1
		self.ser.xonxoff = 0
		self.ser.rtscts = 0
		self.auto = 1
		self.e_stop = 0
		self.gear = 0
		self.speed = 0
		self.brake = 1
		self.steering = 0
		self.alive = 0
		
		rospy.init_node('erp42_sub', anonymous=True)
		rospy.Subscriber('/erp42/speed', Float64, self.speedCallback)
		rospy.Subscriber('/erp42/brake', UInt8, self.brakeCallback)
		rospy.Subscriber('/erp42/gear', UInt8, self.gearCallback)
		rospy.Subscriber('/erp42/auto', UInt8, self.autoCallback)
		rospy.Subscriber('/erp42/e_stop', UInt8, self.e_stopCallback)
		rospy.Subscriber('/erp42/steer', Float64, self.ctrl_Callback)

		self.speedpub = rospy.Publisher('/speed_status', Int16 , queue_size = 10)
		self.steerpub = rospy.Publisher('/steering_status', Int16 , queue_size = 10)
	def sub_to_serial(self):
		self.create_erp42_cmd_packet()
		self.send_packet()
		self.receive_packet()
		# print('{} {} {} {} {} {} {}'.format(self.auto, self.e_stop, self.gear, self.speed, self.brake, self.steering, self.alive))
		
		rospy.sleep(0.1)

	def speedCallback(self, msg):
		self.speed = msg.data

	def brakeCallback(self, msg):
		self.brake = msg.data

	def gearCallback(self, msg):
		self.gear = msg.data

	def autoCallback(self, msg):
		self.auto = msg.data

	def e_stopCallback(self, msg):
		self.e_stop = msg.data

	def ctrl_Callback(self, msg):
		self.steering = msg.data

	def create_erp42_cmd_packet(self):
		fmt = '>BBBBBBHhBBBB'
		self.packet_bytes_cmd = struct.pack(fmt, 0x53, 0x54, 0x58
			,self.auto, self.e_stop, self.gear, int(self.speed*10), int(self.steering*71), self.brake, self.alive
			,0xD, 0xA)
		self.aux_data = [self.auto, self.e_stop, self.gear, self.speed, self.steering, self.brake, self.alive]
		print(self.aux_data)
		self.alive += 1
		if self.alive == 256:
			self.alive = 0


	def send_packet(self):
		# self.ser.open()
		self.ser.write(self.packet_bytes_cmd)
		self.ser.write(b'\n')
		# self.ser.close()

	def receive_packet(self):
		self.read = self.ser.readline()
		print('read:', self.read)
		if self.read[10:14] != '':
			steer=struct.unpack('h',self.read[8:10])
			print(steer)
			speed=struct.unpack('H',self.read[6:8])
			encoder=struct.unpack('i',self.read[11:15])
			print(encoder[0])
			live=struct.unpack('B',self.read[15])
			print(live[0])
			self.speedpub.publish(speed[0])
			self.steerpub.publish(steer[0])

if __name__ == '__main__':

	erp42_serial = ERP42_Serial()
	try:
		while not rospy.is_shutdown():
			erp42_serial.sub_to_serial()

	except Exception as e:
		print(e)
