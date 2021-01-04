#!/usr/bin/env python
import rospy
#import sys
from pylab import *
import numpy as np
import time
from matplotlib import pyplot as plt
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from frame_detection.msg import frame
#import tf, geometry_msgs, tf2_ros
#from tf import TransformBroadcaster
#import sensor_msgs.point_cloud2
import sensor_msgs.point_cloud2 as pc2
#from sensor_msgs.msg import PointCloud2

def callback(value):
	global val
	val = value


def cam_frame(data):
	rate = rospy.Rate(10)
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	l_h = 20 #max_value_H*0.1
	l_s = 70 #max_value*0.3
	l_v = 70 #max_value*0.35
	u_h = 50 #max_value_H*0.25
	u_s = 255
	u_v = 255

	lower_red = np.array([l_h, l_s, l_v])
	upper_red = np.array([u_h, u_s, u_v])

	mask = cv2.inRange(hsv, lower_red, upper_red)
	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.dilate(mask, kernel)

	if int(cv2.__version__[0]) > 3:
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(cnt)
		if (150000 > area > 2000):
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			width = box[1][0]-box[2][0]
			height = box[0][1]-box[1][1]
			if (width<630 and (width/height < 3) and (height/width < 1.5) and (box[0][1] < 476) and (box[3][1] < 476)):
				imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
				a1,b1 = box[0][0]-4,box[0][1]-4
				a2,b2 = box[1][0]-4,box[1][1]+4
				a3,b3 = box[2][0]+4,box[2][1]+4
				a4,b4 = box[3][0]+4,box[3][1]-4
				for p in pc2.read_points(val, field_names = ("x", "y", "z"), skip_nans=False):
					if (i==(640*(b1-1))+a1):
						x1,y1,z1 = p[0],p[1],p[2]
						break
					i+=1
				i=0
				for p in pc2.read_points(val, field_names = ("x", "y", "z"), skip_nans=False):
					if (i==(640*(b2-1))+a2):
						x2,y2,z2 = p[0],p[1],p[2]
						break
					i+=1
				i=0
				for p in pc2.read_points(val, field_names = ("x", "y", "z"), skip_nans=False):
					if (i==(640*(b3-1))+a3):
						x3,y3,z3 = p[0],p[1],p[2]
						break
					i+=1
				i=0
				for p in pc2.read_points(val, field_names = ("x", "y", "z"), skip_nans=False):
					if (i==(640*(b4-1))+a4):
						x4,y4,z4 = p[0],p[1],p[2]
						break
					i+=1
				i=0
				if (np.abs(z1-z2)<0.3 and np.abs(z2-z3)<0.3 and np.abs(z3-z4)<0.3 and np.abs(z1-z4)<0.3):
					fx=(x1+x2+x3+x4)/4
					fy=(y1+y2+y3+y4)/4
					fz=(z1+z2+z3+z4)/4

					ps = PointStamped()
					ps.header.frame_id = "r200link"
					ps.header.stamp = rospy.Time(0)
					ps.point.x = fx
					ps.point.y = fy
					ps.point.z = fz
					mat = listener.transformPoint("/world", ps)
					rospy.loginfo(mat)

					if(3.265<mat.z<3.285):
						# point = Point()
						# point.x = mat.x
						# point.y = mat.y
						# point.z = mat.z
						if(3.9<mat.x<4.1):
							col[0] = mat
						elif(4.9<mat.x<5.1):
							col[1] = mat
						elif(5.9<mat.x<6.1):
							col[2] = mat
						elif(6.9<mat.x<7.1):
							col[3] = mat
						elif(7.9<mat.x<8.1):
							col[4] = mat
						elif(8.9<mat.x<9.1):
							col[5] = mat
						elif(9.9<mat.x<10.1):
							col[6] = mat
						elif(10.9<mat.x<11.1):
							col[7] = mat
						elif(11.9<mat.x<12.1):
							col[8] = mat
						elif(12.9<mat.x<13.1):
							col[9] = mat
						elif(13.9<mat.x<14.1):
							col[10] = mat
						elif(14.9<mat.x<15.1):
							col[11] = mat
						elif(15.9<mat.x<16.1):
							col[12] = mat
						elif(16.9<mat.x<17.1):
							col[13] = mat
						elif(17.9<mat.x<18.1):
							col[14] = mat
						frame_list.centers = col
						pub.publish(frame_list)
						rate.sleep()



	cv2.imshow('image',img)
	cv2.waitKey(30)



if __name__ == '__main__':
	rospy.init_node('world_coordinate', anonymous=True)
	global frame_list
	global col
	frame_list = frame()
	point = Point()
	point.x = 0.0
	point.y = 0.0
	point.z = 0.0
	col = [point]*15
	frame_list.centers = col
	rospy.Subscriber("/r200/rgb/image_raw", Image, cam_frame)
	rospy.Subscriber("/r200/depth/points", PointCloud2, callback)
	pub = rospy.Publisher('frame_center/position', frame, queue_size=100)
	rospy.spin()