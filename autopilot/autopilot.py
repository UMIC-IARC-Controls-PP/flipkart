#!/usr/bin/env python
# ROS python API
import rospy
import time

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from utils.offboard import mavcon
import numpy as np
import pcl
#import pcl_helper
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import tf2_ros

class autopilot:
	
	def __init__(self):
		self.isright = False
		self.isleft = False
		self.currentframe = 0
		self.nextframe = 1
		self.framecount = 0
		
		self.currentposex = 0
		self.currentposey = 0
		self.currentposez = 0
		self.currentoreox = 0
		self.currentoreoy = 0
		self.currentoreoz = 0
		self.currentoreow = 1
		

		self.rate = rospy.Rate(20.0)
		rospy.Subscriber('/mavros/global_position/local', Odometry, self.poseCallback)


	def detectFrames(self):		# Output is self.detectedframe.x/y/z
		pass

	def checkFrame(self):		# Output is self.checkedframe.x/y/z
		pass 

	def getFrameNum(self):
		if ((self.currentpose.x > 1) and (self.currentpose.x < 2)) :
			self.currentframe = 1
		elif ((self.currentpose.x > 2) and (self.currentpose.x < 3)) :
			self.currentframe = 2
		elif ((self.currentpose.x > 3) and (self.currentpose.x < 4)) :
			self.currentframe = 3
		elif ((self.currentpose.x > 4) and (self.currentpose.x < 5)) :
			self.currentframe = 4
		elif ((self.currentpose.x > 5) and (self.currentpose.x < 6)) :
			self.currentframe = 5
		elif ((self.currentpose.x > 6) and (self.currentpose.x < 7)) :
			self.currentframe = 6
		elif ((self.currentpose.x > 7) and (self.currentpose.x < 8)) :
			self.currentframe = 7
		elif ((self.currentpose.x > 8) and (self.currentpose.x < 9)) :
			self.currentframe = 8
		elif ((self.currentpose.x > 9) and (self.currentpose.x < 10)) :
			self.currentframe = 9
		elif ((self.currentpose.x > 10) and (self.currentpose.x < 11)) :
			self.currentframe = 10
		elif ((self.currentpose.x > 11) and (self.currentpose.x < 12)) :
			self.currentframe = 11
		elif ((self.currentpose.x > 12) and (self.currentpose.x < 13)) :
			self.currentframe = 12
		elif ((self.currentpose.x > 13) and (self.currentpose.x < 14)) :
			self.currentframe = 13
		elif ((self.currentpose.x > 14) and (self.currentpose.x < 15)) :
			self.currentframe = 14
		elif ((self.currentpose.x > 15) and (self.currentpose.x < 16)) :
			self.currentframe = 15

		self.nextframe = self.currentframe + 1


	def moveSideways(self, arg):
		mavcon.gotopose(self.currentpose.x, arg, self.currentpose.z)

	def incrementCurrentFrame(self):
		self.framecount = self.framecount + 1

	def explore(self):
		self.frameposeerror.x = self.detectedframe.x - self.checkedframe.x
		self.frameposeerror.y = self.detectedframe.y - self.checkedframe.y
		self.frameposeerror.z = self.detectedframe.z - self.checkedframe.z
		
		while True:
			
			if self.currentpose.y - self.detectedframe.y > 0:
				self.isright = True
			if self.currentpose.y - self.detectedframe.y < 0:
				self.isleft = True

			
			if ((self.frameposeerror.x < 0.1) and (self.frameposeerror.y < 0.1) and (self.frameposeerror.z < 0.1)):
				break
			
			else:
				if isright:
					self.moveSideways(self, self.detectedframe.y)
					self.detectFrames()
					self.checkFrame()
				
				elif isleft:
					self.moveSideways(self, self.detectedframe.y)
					self.detectFrames()
					self.checkFrame()

			rate.sleep()

	def poseCallback(self, msg):
		self.currentposex = msg.pose.pose.position.x
		self.currentposey = msg.pose.pose.position.y
		self.currentposez = msg.pose.pose.position.z
		self.currentoreox = msg.pose.pose.orientation.x
		self.currentoreoy = msg.pose.pose.orientation.y
		self.currentoreoz = msg.pose.pose.orientation.z
		self.currentoreow = msg.pose.pose.orientation.w
	
	'''
	def planner(self, pointlist):
		self.path = []
		for point in pointlist:
			x = point[0] - 0.5
			y = point[1]
			z = point[2]
			self.path.append([x, y, z])
		return self.path
	'''

	def givePoints(self, wp):
		self.wpnew = []
		point1 = [wp[0] - 0.5, wp[1], wp[2]]
		point2 = [wp[0] - 0.5, wp[1], wp[2]]
		point3 = [wp[0] - 0.5, wp[1], wp[2]]
		self.wpnew.append(point1)
		self.wpnew.append(point2)
		self.wpnew.append(point3)
		return self.wpnew

def main():
	rospy.init_node('autopilot')
	atplt = autopilot()
	mvc = mavcon()

	mvc.setarm(1)
	time.sleep(2)
	mvc.offboard()
	mvc.gotopose(0, 0, 3)

	while True:
		atplt.detectFrames()
		atplt.checkFrame()
		atplt.explore()
		wpnew = atplt.givePoints(wp)
		atplt.getFrameNum()
		gotopose(wpnew[0][0], wpnew[0][1], wpnew[0][2])
		gotopose(wpnew[1][0], wpnew[1][1], wpnew[1][2])
		gotopose(wpnew[2][0], wpnew[2][1], wpnew[2][2])
		self.framecount += 1

	rospy.spin()


if __name__ == '__main__':
	main()
