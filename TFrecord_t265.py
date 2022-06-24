#!/usr/bin/python
# -*- coding: utf-8 -*-

#####################################################
##                   TFrecord.py                   ##
#####################################################
#Takuma MORI, 2020

#####################################################
## This program outputs and records the positional ##
## relationship between the 2 TFs.                 ##
#####################################################

# First import the library
import rospy
import tf
import numpy as np
import time
import datetime

# Define TF frame name
TF_from = "/camera_odom_frame"
TF_to =  "/camera_link"
# File name and path
now = datetime.datetime.now()
FILE_NAME = "/home/nakajima/catkin_ws/src"  + now.strftime('%Y%m%d-%H%M%S') + ".txt"
# Define sleep time
start_time = 0.0
# Define sleep time
DEFAULT_SLEEP_TIME = 0.1

# Choose how to express pose and rotation
'''
The default output is Eulerian expression (set to false).
In the case of Quaternion expression, set to true.
'''
Quaternion = "false"


class TFRecordNode(object):

	def __init__(self):

		# Open file
		self.r = open(FILE_NAME, 'w')

		if Quaternion == "false":
			self.r.write("time(s), x(m)   , y(m)   , z(m)   , roll(rad), pitch(ra), yaw(rad)")
			self.r.write("\n")
			print("time(s), x(m)   , y(m)   , z(m)   , roll(rad), pitch(ra), yaw(rad)")
			print("000.000, -00.000, -00.000, -00.000, -0.000000, -0.000000, -0.000000")			

		elif Quaternion == "true":
			self.r.write("time(s), x(m)   , y(m)   , z(m)   , rot_x    , rot_y    , rot_z    , rot_w    ")
			self.r.write("\n")
			print("time(s), x(m)   , y(m)   , z(m)   , rot_x    , rot_y    , rot_z    , rot_w    ")
			#print("000.000, -00.000, -00.000, -00.000, -0.000000, -0.000000, -0.000000, -0.000000")

		# Creating subscriber, subscribe (receive) from topic tf
		listener = tf.TransformListener()
		while not rospy.is_shutdown():
			try:
				(trans,rot) = listener.lookupTransform(TF_from, TF_to, rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			self.callback(trans,rot)

			time.sleep(DEFAULT_SLEEP_TIME)


	# Subscriber method
	def callback(self, tf_trans, tf_rot):

		global start_time

		if start_time == 0.0:
			start_time = time.time()
			self.time = 0.000
		else:
			self.time = time.time() - start_time

		if Quaternion == "false":
			euler = self.quaternion_to_euler(tf_rot)
			self.r.write("{0:07.3f}, {1[0]: 07.3f}, {1[1]: 07.3f}, {1[2]: 07.3f}, {2[0]: 09.6f}, {2[1]: 09.6f}, {2[2]: 09.6f}".format(self.time, tf_trans, euler))
			self.r.write("\n")
			print("{0:07.3f}, {1[0]: 07.3f}, {1[1]: 07.3f}, {1[2]: 07.3f}, {2[0]: 09.6f}, {2[1]: 09.6f}, {2[2]: 09.6f}".format(self.time, tf_trans, euler))	

		elif Quaternion == "true":
			self.r.write("{0:07.3f}, {1[0]: 07.3f}, {1[1]: 07.3f}, {1[2]: 07.3f}, {2[0]: 09.6f}, {2[1]: 09.6f}, {2[2]: 09.6f, {2[3]: 09.6f}".format(self.time, tf_trans, tf_rot))
			self.r.write("\n")
			print("{0:07.3f}, {1[0]: 07.3f}, {1[1]: 07.3f}, {1[2]: 07.3f}, {2[0]: 09.6f}, {2[1]: 09.6f}, {2[2]: 09.6f, {2[3]: 09.6f}".format(self.time, tf_trans, tf_rot))	


	def quaternion_to_euler(self, quaternion):
		"""Convert Quaternion to Euler Angles

		quarternion: geometry_msgs/Quaternion
		euler: geometry_msgs/Vector3
		"""
		e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
		return e[0], e[1], e[2]
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle


	def update_parameters(self):
		"""
		Parameters modification.
		Updates (modifies) default parameters with values from ROS server (in case there are some).
		"""


# Main program
if __name__ == '__main__':

	try:
		# Node initialization (node name)
		rospy.init_node('TF_recording', anonymous=True)

		# Creating object, which subscribes (receives) data from sensors and publish (send) them to the robot control
		node_tf_record = TFRecordNode()

		# spin() keeps python from exiting until this node is stopped
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
