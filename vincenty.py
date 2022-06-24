#!/usr/bin/python3
# -*- coding: utf-8 -*-

#####################################################
##                   vincenty.py                   ##
#####################################################
#Takuma MORI, 2020

#####################################################
##  This program calculates an direction from      ##
##  longitude and latitude.                        ##
##  (This program uses the inverse solution of     ##
##   the 'vincenty method')                        ##
##  For this program to run, 'vincenty_func.py'    ##
##  must be in the same directory.                 ##
#####################################################

# First import the library
import rospy
import tf
from sensor_msgs.msg import NavSatFix
import numpy as np
import math
import time
from geometry_msgs.msg import Quaternion
import vincenty_func
from calcxy import calc_xy

# Define publisher, subscriber topic name
DEFAULT_SUBSCRIBER_TOPIC = "/ublox/fix"
# Define TF name
TF_PARENT = "world"
TF_CHILD = "GNSS_DIR"
# Define sleep time
DEFAULT_SLEEP_TIME = 0.1


class MainNode(object):

	def __init__(self):

		# From ROS parameter (set in launch file)
		subscriber_topic = rospy.get_param("~subscriber_topic", DEFAULT_SUBSCRIBER_TOPIC)

		# Creating subscriber, subscribe (recieve) from topic
		self.sub = rospy.Subscriber(subscriber_topic, NavSatFix, self.callback)

		# Creating broadcaster, publish (send) to TF
		self.br = tf.TransformBroadcaster()

		# Creating a list to keep longitudes
		self.lon = []
		# Creating a list to keep latitudes
		self.lat = []

		# Initial x position
		self.init_px = 0.0
		# Initial y position
		self.init_py = 0.0


	# Subscriber method
	def callback(self, data):

		# The latest data from GNSS
		# x:longitude, y:latitude
		x,y = (data.longitude, data.latitude)

		# Set initial x,y position
		if self.init_px == 0.0:
			self.init_px = x

		if self.init_py == 0.0:
			self.init_py = y

		# Use 10 data for each longitudes, latitudes
		# Let the average of the first 5 data be past point and the average of the second 5 data be the current point

		if len(self.lon) == 0 :
			for i in range(10):
				self.lon.append(x)
		else:
			del self.lon[0]
			self.lon.append(x)

		if len(self.lat) == 0 :
			for i in range(10):
				self.lat.append(y)
		else:
			del self.lat[0]
			self.lat.append(y)

		lon1 = (self.lon[0]+self.lon[1]+self.lon[2]+self.lon[3]+self.lon[4]) / 5
		lon2 = (self.lon[5]+self.lon[6]+self.lon[7]+self.lon[8]+self.lon[9]) / 5
		lat1 = (self.lat[0]+self.lat[1]+self.lat[2]+self.lat[3]+self.lat[4]) / 5
		lat2 = (self.lat[5]+self.lat[6]+self.lat[7]+self.lat[8]+self.lat[9]) / 5

		# Calculate the direction from 2 points
		e_direction = self.calc_direction(lon1, lat1, lon2, lat2)
		e_direction = [0, 0, e_direction]

		# Convert Euler Angles to Quaternion
		self.trans = self.euler_to_quaternion(e_direction)

		# Caluculating X,Y position
		py, px = calc_xy(y, x, self.init_py, self.init_px)
		self.pos = (px , py, 0.0)

		# Invoking method for publishing message
		self.publish_message()

		#time.sleep(DEFAULT_SLEEP_TIME)


	def calc_direction(self, x1, y1, x2, y2):
		"""
		* Calculate the direction from 2 points *
		Refer to the following URL
		'https://qiita.com/r-fuji/items/99ca549b963cedc106ab'

		The result is the direction from pointA to pointB.
		x1, y1: longitude, latitude of pointA
		x2, y2: longitude, latitude of pointB
		North: 0, West: PI/2, South: PI, East: 3PI/2
		Result is output in the range of -PI to PI
		"""
		try:
			result = vincenty_func.vincenty_inverse(y1, x1, y2, x2, 1)
		except:
			pass

		direction = math.radians(result['azimuth1'])

		if direction > math.pi:
			direction -= (2 * math.pi)
		elif direction < -1 * math.pi:
			direction += (2 * math.pi)
		return direction


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
