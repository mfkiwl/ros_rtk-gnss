#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------------------------------
# controller.py for PWM control
#-------------------------------------

import rospy
from geometry_msgs.msg import Twist
import sys
import RPi.GPIO as GPIO
import time

#GPIO initial setting
def init():
	GPIO.cleanup()
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(channel_list,GPIO.OUT)
	del list[:]
	for i in channel_list:
		list.append(GPIO.PWM(i,100))

def listener():
	received_twist = None
	rospy.init_node('GPIO_controller')
	rospy.Subscriber('cmd_vel', Twist, callback)
	rospy.spin()

def callback(message):
	received_data = message
	v = received_data.linear.x#(m/s)
	omega = received_data.angular.z#(rad/s)
	gpio_control(v,omega)

def gpio_control(v,omega):
# Stop
	if v == 0.0:
		val_pwm = (-1) * omega / max_vel_theta
		list[4].start(0)
		list[5].start(0)
		list[0].start(100)
		list[1].start(0)
		list[2].start(100)
		list[3].start(0)
		time.sleep(sleep_time)
		print('L:   0% R:   0%  => STOP')
		
# Straight
	elif -0.5 < omega < 0.5:
		list[4].start(R_max)
		list[5].start(L_max)
		list[0].start(100)
		list[1].start(0)
		list[2].start(100)
		list[3].start(0)
		time.sleep(sleep_time)
		print('L: 100% R: 100% => STRAIGHT')


# Right
	elif -0.5 >= omega:
		omega = -1 * omega
		val_pwm = 100 - (omega * 66)
		list[4].start(val_pwm*R_max)
		list[5].start(L_max)
		list[0].start(100)
		list[1].start(0)
		list[2].start(100)
		list[3].start(0)
		time.sleep(sleep_time)
		print('L: 100% R: {:>3}% => RIGHT'.format(val_pwm))

# Left
	elif omega >= 0.5:
		omega = omega
		val_pwm = 100 - (omega * 66)
		if 
		list[4].start(R_max)
		list[5].start(val_pwm*L_max)
		list[0].start(100)
		list[1].start(0)
		list[2].start(100)
		list[3].start(0)
		time.sleep(sleep_time)
		print('L: {:>3}% R: 100% => LEFT'.format(val_pwm))


# Main Program
if __name__ == '__main__':

# Define PIN number (GPIO.BOARD)
# IN1,IN2,PWM1 are for right motor
# IN3,IN4,PWM2 are for left motor

	IN1 = 11
	IN2 = 12
	IN3 = 13
	IN4 = 15
	PWM1 = 16
	PWM2 = 18
	channel_list = [IN1,IN2,IN3,IN4,PWM1,PWM2]
	list = []

# Set motor max PWM for going straight
	R_max = 100
	L_max = 100

# Set motor minimum PWM for going straight
#	R_min = 40
#	L_min = 40

# xxx
#	max_vel_x = 0.037#(m/s)
#	max_vel_theta = 0.357#(rad/s)
#	min_vel_theta = 0.26#(rad/s)

# Sleep time
	sleep_time = 0.5

# GPIO initial setting
	init()

# Listener function
	listener()

# Test execution using rostopic pub
# rostopic pub cmd_vel geometry_msgs/Twist '[x, 0.0, 0.0]' '[0.0, 0.0, z]'
