#!/usr/bin/python3
# -*- coding: utf-8 -*-

#####################################################
##            cmdvel_controller_t265.py            ##
#####################################################
#Takuma MORI, 2019

# First import the library
import rospy
import tf
from geometry_msgs.msg import Twist
import numpy as np
import time
import math

# Define publisher, subscriber topic name
DEFAULT_PUBLISHER_TOPIC = "/auto_velocity"
# Define sleep time
DEFAULT_SLEEP_TIME = 0.5
# Define cmd_vel message
DEFAULT_LINEAR_X = 0.0
DEFAULT_LINEAR_Y = 0.0
DEFAULT_LINEAR_Z = 0.0
DEFAULT_ANGULAR_X = 0.0
DEFAULT_ANGULAR_Y = 0.0
DEFAULT_ANGULAR_Z = 0.0


# From ROS parameter (set in launch file)
#DEFAULT_PUBLISHER_TOPIC = rospy.get_param("~pub_topic", "/cmd_vel")
#DEFAULT_SUBSCRIBER_TOPIC = rospy.get_param("~sub_topic", "/tf")


class CmdVelPubNode(object):

	def __init__(self):
	

		publisher_topic = ""

		if not rospy.get_param("~publisher_topic", publisher_topic):
			publisher_topic = DEFAULT_PUBLISHER_TOPIC

		# Creating publisher, publish (send) to topic cmdVelTopic
		self.pub = rospy.Publisher(publisher_topic, Twist, queue_size=0.5)

		# Creating subscriber, subscribe (receive) from topic tf
		listener = tf.TransformListener()
		while not rospy.is_shutdown():
			try:
				(trans,rot) = listener.lookupTransform('/world', '/GNSS_DIR', rospy.Time(0)) #原点と自分の位置との相対位置
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			
			self.callback(trans,rot)
			


	# Subscriber method
	def callback(self, tf_trans, tf_rot):
		# Caluculate the closest point
		tf_trans.pop(2)
		path = [[13.890853867400438, 2.9794624675305754], [13.890853867400438, 2.9794624675305754], [13.890853867400438, 2.9794624675305754], [13.890853867400438, 2.9794624675305754], [13.890853867400438, 2.9794624675305754], [13.890853867400438, 2.9794624675305754]] #目標地点座標[前後,左右]
		p0 = np.array(tf_trans)#現在地xy[xxx,yyy]
		ps = np.array(path)#目標点をnumpyに
		L = np.array([])#空の配列
		for i in range(ps.shape[0]):#目標地点の配列の数だけ繰り返す
			L = np.append(L,np.linalg.norm(ps[i]-p0))#ベクトルのノルムを計算した配列
		# np.argmin(L) represents the index of ps with the smallest distance

		# Stop Judgement
		if set(ps[np.argmin(L)]) == set(ps[-1]):#目標点の最後と一番近い目標点が等しくなると止まる
			self.stp_judgement = 1
		else:
			self.stp_judgement = 0

		# Redefine variables
		current_point = tf_trans #現在地
		if self.stp_judgement == 0: 
			target_point = ps[np.argmin(L) + 1] #一番近いところの次の点をターゲットポイント

			# Conversion from quaternion to Euler angle
			current_yaw = self.quaternion_to_euler(tf_rot)[2] #クォータニオンからオイラー角に変換してヨー角

			# Calculation yaw angle from current location to target location
			target_yaw = self.calc_target_yaw(current_point, target_point) #現在地とターゲットポイントとの角度

			# The angle that the robot should move
			self.fin_yaw = target_yaw - current_yaw #ターゲットポイントと現在のロボットの角度との差
	
			if self.fin_yaw > math.pi:
				self.fin_yaw -= (2 * math.pi)
			elif self.fin_yaw < -1 * math.pi:
				self.fin_yaw += (2 * math.pi)

		# Invoking method for publishing message
		self.publish_message() #ロボットにだす指示（速度）のパブリッシュ

		time.sleep(0.5)


	def quaternion_to_euler(self, quaternion):
		e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
		return e[0], e[1], e[2]
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle

	def euler_to_quaternion(roll,pitch,yaw):#roll,pitch,yaw
		q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		return q[0], q[1], q[2], q[3] #x,y,z,w

	def calc_target_yaw(self, cur, tar):
		yaw = math.atan2(tar[1] - cur[1], tar[0] - cur[0])
		return yaw


	# Publisher method
	def publish_message(self):
		msg = Twist()

		if self.stp_judgement == 1:
			msg.linear.x = 0.0
		elif self.stp_judgement == 0:
			msg.linear.x = 1.0

		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = self.fin_yaw

		# Publishing message
		self.pub.publish(msg)


	def update_parameters(self):
		"""
		Parameters modification.
		Updates (modifies) default parameters with values from ROS server (in case there are some).
		"""


# Main program
if __name__ == '__main__':
	try:
		# Node initialization (node name)
		rospy.init_node('cmdvel_publisher', anonymous=True)

		# Creating object, which subscribes (receives) data from sensors and publish (send) them to the robot control
		node_cmd_vel = CmdVelPubNode()

		# spin() keeps python from exiting until this node is stopped
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
