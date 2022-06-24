#!/usr/bin/python3
# -*- coding: utf-8 -*-

#####################################################
##             depth_gridPlotXZ_hv.py              ##
#####################################################
#Takuma MORI, 2020

#####################################################
##  This program converts PointCloud2 messages to  ##
##  PointCloud<T>.                                 ##
##  And output the depth map as a scatter plot.    ##
##  In addition, it divides the point cloud into   ##
##  a grid and calculate the angle of the pathway. ##
##  This uses the maximum value for each row to    ##
##  calculate it.                                  ##
#####################################################

# First import the library
import rospy
import numpy as np
import math
from scipy.signal import argrelextrema
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import sensor_msgs.point_cloud2 as pc2
import time
import rotation_array_func
import ros_numpy

"""
How to install 'pcl' to Ubuntu
--> $ sudo pip install python-pcl
"""

# Define subscribe topic name
SUB_TOPIC = "/camera/depth/color/points"

# Define sleep time
DEFAULT_SLEEP_TIME = 0.5

first_loop = True
start = time.time()

# Limitations for Caluculation X
min_x = -1
max_x = 1
min_y = -0.4
max_y = -0.1
min_z = 0
max_z = 3

# Depth ColorMap Limitations (NOT using for Caluculation)
Vmin_z = 0
Vmax_z = 4

# Number of grid divisions ( n*n )
mesh_num = 100

# The ranges of dividing into grids
# The ranges of Displaying Graphs
# In x,z, it's recommended that the difference between the minimum and maximum values should be the same.
Dmin_x = -2
Dmax_x = Dmin_x * -1
Dmin_z = 0
Dmax_z = 4

del_mx = (Dmax_x - Dmin_x) / mesh_num
del_mz = (Dmax_z - Dmin_z) / mesh_num

class NodeClass(object):

	def __init__(self):
		# Creating subscriber, subscribe (recieve) from topic
		self.sub = rospy.Subscriber(SUB_TOPIC, PointCloud2, self.callback, queue_size=1)

		# Creating publisher, publish (send) to topic
		self.pub1 = rospy.Publisher("angle_float32", Float32, queue_size=1)

		# Creating publisher, publish (send) to topic
		self.pub2 = rospy.Publisher("center_float32", Float32, queue_size=1)

	# Subscriber method
	def callback(self,data):
		array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=True)[::5] #回転させたりFPS上げたいときはこれ
		array[:,1] = (-1) * array[:,1]

		# Function to rotate an array (optional)
		#rotation_array_func.rotation_array(array, 5)#degree

		array = self.limit_xyz(array)
		array = np.delete(array,1,1)#高さ情報削除

		mesh_array = self.mesh(array)
		mv_array, pc_pathway = self.mv_mesh(mesh_array)

		if len(pc_pathway) == 0:
			a = np.nan
			b = np.nan
		else:
			a, b = self.ls_line(pc_pathway)

		a = -a
		b = b + 0.5
		angle = math.atan(a)
		angle_deg = round(math.degrees(angle), 0)

		a = round(a, 3)
		b = round(b/25 -2, 3)

		# Invoking method for publishing message
		self.publish_message1(angle)
		self.publish_message2(b)

		"""
		global start
		elapsed_time = time.time() - start
		print("[DepthPlot] FPS: {}".format(1/elapsed_time))
		start = time.time()
		"""
		
	# Limit the range of X, Y and Z values to caluculate X
	def limit_xyz(self, array):
		limzy_index = np.where((array[:, 0] > min_x) & (array[:, 0] < max_x) & (array[:, 1] > min_y) & (array[:, 1] < max_y) & (array[:, 2] > min_z) & (array[:, 2] < max_z))
		array = array[limzy_index]
		return array

	# Create an n*n array with 1,0 representing the presence of points in the lattice
	#データを格子点スケールに変換して小数点切り捨てしてそのままそのインデックスのところを１
	def mesh(self,array):
		m_array = np.zeros((mesh_num, mesh_num))
		m_array_z = array.T[1]
		m_array_x = array.T[0]

		mz = np.floor(m_array_z/del_mz).astype(np.int32)
		mx = np.floor(m_array_x/del_mx + 50).astype(np.int32)

		m_array[mz,mx]=1

		return m_array

	# Calculating extracting maxima to the grid array
	def mv_mesh(self, m_array):
		mv_array = np.zeros((mesh_num, mesh_num))
		pc_pathway = []
		for i in range(5):
			index1 = np.where(m_array==1)[0], np.where(m_array==1)[1]+i-2
			index2 = np.where(m_array==0)[0], np.where(m_array==0)[1]+i-2
			over1 = np.where((index1[1] <= -1) | (index1[1] >= 100))
			over2 = np.where((index2[1] <= -1) | (index2[1] >= 100))
			index1 = np.delete(index1[0],over1,0),np.delete(index1[1],over1,0)
			index2 = np.delete(index2[0],over2,0),np.delete(index2[1],over2,0)
			np.add.at(mv_array,index1,1)
			np.add.at(mv_array,index2,-1)

		maxIndex = argrelextrema(mv_array,np.greater_equal,axis=1,order=25)#極大値インデックス
		minIndex = argrelextrema(mv_array,np.less_equal,axis=1,order=25)#極大値インデックス
		a = np.where(mv_array[maxIndex]>0)
		b = np.where(mv_array[minIndex]<0)
		maxIndex = maxIndex[0][a],maxIndex[1][a] #選別済みindex
		minIndex = minIndex[0][b],minIndex[1][b]
		maxindex_z = np.unique(maxIndex[0]) #Z座標配列
		maxIndex = np.split(maxIndex[1],np.where(np.diff(maxIndex[0])!=0)[0]+1)#Z行1~100の極大値インデックスの集合
		minIndex = np.split(minIndex[1],np.where(np.diff(minIndex[0])!=0)[0]+1)


		for j in range(len(maxIndex)):
			z = maxindex_z[j]
			cont_max_index = np.split(maxIndex[j], np.where(np.diff(maxIndex[j]) != 1)[0]+1) #行ごとに,連続する極大値ni分割

			if len(cont_max_index) == 2:
				cont_min_index = np.split(minIndex[z], np.where(np.diff(minIndex[z]) != 1)[0]+1)
				max_ave1 = np.average(cont_max_index[0])
				max_ave2 = np.average(cont_max_index[1])

				for i in cont_max_index[0]:
					mv_array[z, i] = 10
				for i in cont_max_index[1]:
					mv_array[z, i] = 10

				max_min_max = []
				for check in range(len(cont_min_index)):
					min_ave = np.average(cont_min_index[check])

					if min_ave > max_ave1 and min_ave < max_ave2:
						max_min_max.append(check)

				if len(max_min_max) == 1:
					way_point = np.average(cont_min_index[max_min_max[0]])
					for i in cont_min_index[max_min_max[0]]:
						mv_array[z, i] = -10

					pc_pathway.append([z, way_point])

		pc_pathway = np.array(pc_pathway)

		# delete exaggerated values
		if len(pc_pathway) != 0:
			delete_list = []

			a, b = self.ls_line(pc_pathway)
			b = b + 0.5

			for i in range(len(pc_pathway)):
				if abs(pc_pathway[i,1] - (a*(pc_pathway[i,0]) + b)) > 10:
					delete_list.append(i)
					delete_list.reverse()

			if len(delete_list) != 0:
				for i in delete_list:
					pc_pathway = np.delete(pc_pathway, i, 0)

		return mv_array, pc_pathway

	# least-squares method
	def ls_line(self, pc):
		x = pc[:,0]
		y = pc[:,1]

		a = ((x * y).mean() - (x.mean() * y.mean())) / ((x ** 2).mean() - x.mean() ** 2)
		b = -(a * x.mean()) + y.mean()

		# Note that the x and y axes are swapped compared to the general least squares method
		return a, b

	# Publisher method 1
	def publish_message1(self, angle):
		msg = Float32()

		msg.data = angle

		# Publishing message
		self.pub1.publish(msg)

	# Publisher method 2
	def publish_message2(self, center_dis):
		msg = Float32()

		msg.data = center_dis

		# Publishing message
		self.pub2.publish(msg)


# Main program
if __name__ == '__main__':
	try:
		# Node initialization (node name)
		rospy.init_node('depth_plot', anonymous=True)

		# Creating object, which subscribes (receives) data from topics and publish (send) them to other topics
		class_object = NodeClass()

		# spin() keeps python from exiting until this node is stopped
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
