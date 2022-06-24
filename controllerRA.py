#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-----------------------------------------------
# controllerRA.py for aruduino via Raspberry Pi
# This program is used in Raspberry Pi
#-----------------------------------------------

import rospy
from geometry_msgs.msg import Twist
import sys
import serial
import time


#USB initial setting
def init():
	global ser
	ser = serial.Serial('/dev/ttyUSB0', 115200)

def listener():
	received_twist = None
	rospy.init_node('Serial_controller')
	rospy.Subscriber('auto_velocity', Twist, callback)
	rospy.spin()

def callback(message):
	received_data = message
	v = received_data.linear.x#(m/s)
	omega = received_data.angular.z#(rad/s)
	serial_control(v,omega)

def serial_control(v,omega):
# Stop
	if v == 0.0:
		ser.write('q;')
		time.sleep(sleep_time)
		print('STOP')

# Straight
	elif -0.3 < omega < 0.3:
		ser.write(str.encode('w;'))
		time.sleep(sleep_time)
		print('STRAIGHT')		
# Back
	elif v == -1.0:
		ser.write(str.encode('s;'))
		time.sleep(sleep_time)
		print('BACK')

# Turn Right
	elif -0.4 >= omega:
		ser.write(str.encode('c;'))
		time.sleep(sleep_time)
		print('TURN_RIGHT')

# Turn Left
	elif omega >= 0.4:
		ser.write(str.encode('z;'))
		time.sleep(sleep_time)
		print('TURN_LEFT')

'''
# Right
	elif -0.5 >= omega:
		ser.write(str.encode('d;'))
		time.sleep(sleep_time)
		print('RIGHT')

# Left
	elif omega >= 0.5:
		ser.write(str.encode('a;'))
		time.sleep(sleep_time)
		print('LEFT')
'''

# Main Program
if __name__ == '__main__':

# Sleep time
	sleep_time = 0.01

# USB initial setting
	init()

# Listener function
	listener()

# Test execution using rostopic pub
# rostopic pub cmd_vel geometry_msgs/Twist '[x, 0.0, 0.0]' '[0.0, 0.0, z]'
