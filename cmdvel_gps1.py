#!/usr/bin/python3
# -*- coding: utf-8 -*-


import rospy
import tf
import numpy as np
import math
import time
from calcxy import calc_xy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

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

# Define publisher, subscriber topic name
DEFAULT_SUBSCRIBER_TOPIC = "/ublox/fix"
# Define TF name
TF_PARENT = "world"
TF_CHILD = "GNSS_DIR"
# Define sleep time
DEFAULT_SLEEP_TIME = 0.1

targets_point = [[36.07575271,136.21362955],[36.07575271,136.21362855],[36.07575271,136.21362755],[36.07575271,136.21362655],[36.07575271,136.21362555]]
targets = []

first = True

class MainNode(object):

	def __init__(self):

		# From ROS parameter (set in launch file)
		subscriber_topic = rospy.get_param("~subscriber_topic", DEFAULT_SUBSCRIBER_TOPIC)

		# Creating subscriber, subscribe (recieve) from topic
		self.sub = rospy.Subscriber(subscriber_topic, NavSatFix, self.callback)

		# Creating broadcaster, publish (send) to TF
		self.br = tf.TransformBroadcaster()

		# Initial x position
		self.init_latitude = 0.0
		# Initial y position
		self.init_longitude = 0.0

		self.x = 0
		self.y = 0

		self.fin_yaw = 0


		publisher_topic = ""

		if not rospy.get_param("~publisher_topic", publisher_topic):
			publisher_topic = DEFAULT_PUBLISHER_TOPIC

		# Creating publisher, publish (send) to topic cmdVelTopic
		self.pub = rospy.Publisher(publisher_topic, Twist, queue_size=0.5)


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
		# x:latitude, y:longitude
		self.x,self.y = (data.latitude, data.longitude)

		# Set initial x,y position
		if self.init_latitude == 0.0:
			self.init_latitude = self.x

		if self.init_longitude == 0.0:
			self.init_longitude = self.y
			
		global first

		if first == True:
			self.calc_targets(targets_point)
			first = False

	def calc_targets(self,targets_point):
		for i in range(len(targets_point)):
			x,y = calc_xy(targets_point[i][0], targets_point[i][1], self.init_latitude, self.init_longitude)
			targets.append([x,y])
		print(targets)

	def euler_to_quaternion(self, euler):
		q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle
		return q
		# q[0] is x, q[1] is y, q[2] is z, q[3] is w of Quaternion

	def quaternion_to_euler(self, quaternion):
		e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
		return e[0], e[1], e[2]
		# e[0] is roll, e[1] is pitch, e[2] is yaw angle

	def publish_message(self):
		self.br.sendTransform(self.pos, self.trans, rospy.Time.now(), TF_CHILD, TF_PARENT)

	def mein(self,trans,rot):
		e_direction = self.quaternion_to_euler(rot)[2]
		e_direction = [0, 0, e_direction]

		# Convert Euler Angles to Quaternion
		self.trans = self.euler_to_quaternion(e_direction)

		# Caluculating X,Y position
		py, px = calc_xy(self.x, self.y, self.init_latitude, self.init_longitude)
		self.pos = (px , py, 0.0)

		# Invoking method for publishing message
		self.publish_message()

		self.calc_cmd(self.pos,self.trans)

	def calc_cmd(self, tf_trans, tf_rot):
		
		
			# Caluculate the closest point
		
		tf_trans = list(tf_trans)
		tf_trans.pop(2)
		path = targets #目標地点座標[前後,左右]
		p0 = np.array(tf_trans)#現在地xy[xxx,yyy]
		ps = np.array(path)#目標点をnumpyに
		L = np.array([])#空の配列
		for i in range(ps.shape[0]):#目標地点の配列の数だけ繰り返す
			L = np.append(L,np.linalg.norm(ps[i]-p0))#ベクトルのノルムを計算した配列
		# np.argmin(L) represents the index of ps with the smallest distance

		# Stop Judgement
		if set(ps[np.argmin(L)]) == set(ps[-1]) and np.amin(L) < 1:#目標点の最後と一番近い目標点が等しくなると止まる
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

		self.publish_cmd()

	def calc_target_yaw(self, cur, tar):
		yaw = math.atan2(tar[1] - cur[1], tar[0] - cur[0])
		return yaw

	# Publisher method
	def publish_cmd(self):
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
