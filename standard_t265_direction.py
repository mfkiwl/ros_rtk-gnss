#!/usr/bin/python3
# -*- coding: utf-8 -*-

#####################################################
##              standard_direction.py              ##
#####################################################
#Takuma MORI, 2020

#####################################################
##  This program calculates an direction from      ##
##  longitude and latitude.                        ##
##  (This program assumes that 'the Earth is a     ##
##   sphere')                                      ##
#####################################################

# First import the library
import rospy
import tf
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import time
from geometry_msgs.msg import Quaternion
from calcxy import calc_xy

# Define publisher, subscriber topic name
DEFAULT_SUBSCRIBER_TOPIC = "/ublox/fix"
# Define TF name
TF_PARENT = "world"
TF_CHILD = "GNSS_DIR"
# Define sleep time
DEFAULT_SLEEP_TIME = 0.1

targets_point = [[136.213456780,36.07573310],[136.213456780,36.07573310],[136.213456780,36.07573310]
,[136.213456780,36.07573310],[136.213456780,36.07573310]]
targets = []


class MainNode(object):

	def __init__(self):

		# From ROS parameter (set in launch file)
		subscriber_topic = rospy.get_param("~subscriber_topic", DEFAULT_SUBSCRIBER_TOPIC)

		# Creating subscriber, subscribe (recieve) from topic
		self.sub = rospy.Subscriber(subscriber_topic, NavSatFix, self.callback)

		# Creating broadcaster, publish (send) to TF
		self.br = tf.TransformBroadcaster()

		# Initial x position
		self.init_px = 0.0
		# Initial y position
		self.init_py = 0.0

		self.x = 0
		self.y = 0

		listener = tf.TransformListener()
		while not rospy.is_shutdown():
			try:
				(trans,rot) = listener.lookupTransform('/camera_odom_frame', '/camera_link', rospy.Time(0)) #原点と自分の位置との相対位置
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			self.mein(trans,rot)


	# Subscriber method
	def callback(self, data):

		# The latest data from GNSS
		# x:longitude, y:latitude

		self.x,self.y = (data.longitude, data.latitude)

		# Set initial x,y position
		if self.init_px == 0.0:
			self.init_px = self.x

		if self.init_py == 0.0:
			self.init_py = self.y

		self.calc_targets(targets_point)

	def mein(self,trans,rot):
		e_direction = self.quaternion_to_euler(rot)[2]
		e_direction = [0, 0, e_direction]

		# Convert Euler Angles to Quaternion
		self.trans = self.euler_to_quaternion(e_direction)

		# Caluculating X,Y position
		px, py = calc_xy(self.y, self.x, self.init_py, self.init_px)
		self.pos = (px , py, 0.0)

		# Invoking method for publishing message
		self.publish_message()

		#time.sleep(DEFAULT_SLEEP_TIME)

	def calc_targets(self,targets_point):
		for i in range(len(targets_point)):
			x,y = calc_xy(targets_point[i][1], targets_point[i][0], self.init_py, self.init_px)
			targets.append([x,y])
		print(targets)


	def euler_to_quaternion(self, euler):
		"""
		* Convert Euler Angles to Quaternion *
		euler: geometry_msgs/Vector3
		quaternion: geometry_msgs/Quaternion
		"""
		q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle
		return q
		# q[0] is x, q[1] is y, q[2] is z, q[3] is w of Quaternion

	def quaternion_to_euler(self, quaternion):
		e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
		return e[0], e[1], e[2]
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle



	# Publisher method
	def publish_message(self):

		self.br.sendTransform(self.pos, self.trans, rospy.Time.now(), TF_CHILD, TF_PARENT)


	def update_parameters(self):
		"""
		Parameters modification.
		Updates (modifies) default parameters with values from ROS server (in case there are some).
		"""


# Main program
if __name__ == '__main__':

	try:
		# Node initialization (node name)
		rospy.init_node('GNSS_direction_calculater', anonymous=True)

		# Creating object
		my_node = MainNode()

		# spin() keeps python from exiting until this node is stopped
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
