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
import tf, geometry_msgs, tf2_ros
from tf import TransformBroadcaster
#import sensor_msgs.point_cloud2
import sensor_msgs.point_cloud2 as pc2
import itertools

from itertools import islice
#from sensor_msgs.msg import PointCloud2

def callback(value):
	global val
	val = value

def scale_contour(cnt, scale):
	M = cv2.moments(cnt)
	if M['m00']==0:
		return cnt
	else:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cnt_norm = cnt - [cx, cy]
		cnt_scaled = cnt_norm * scale
		cnt_scaled = cnt_scaled + [cx, cy]
		cnt_scaled = cnt_scaled.astype(np.int32)
		return cnt_scaled

# def pixel_to_depth(h,w,gen):
# 	ptlist = gen
# 	i=(640*h)+w+1
# 	cord = list(itertools.islice(ptlist, i, i+1))
# 	return cord

def cam_frame(data):
	#rate = rospy.Rate(10)
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
	kernel = np.ones((3,3), np.uint8)
	mask = cv2.dilate(mask, kernel)
	kernel2 = np.ones((4,4), np.uint8)
	mask = cv2.erode(mask, kernel2)

	if int(cv2.__version__[0]) > 3:
		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	j = 0
	for cnt in contours:
		cnt = scale_contour(cnt, 1.01)
		#approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(cnt)
		if (150000 > area > 2000):
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int0(box)
			width = box[1][0]-box[2][0]
			height = box[0][1]-box[1][1]
			if (width<630 and (width/height < 3) and (height/width < 1.5) and (box[0][1] < 476) and (box[3][1] < 476)):
				imgs = cv2.drawContours(img, [box], 0, (0,0,255), 2)
				a1,b1 = box[0][0]-1,box[0][1]-1
				a2,b2 = box[1][0]-1,box[1][1]+1
				a3,b3 = box[2][0]+1,box[2][1]+1
				a4,b4 = box[3][0]+1,box[3][1]-1

				#depth = val
				now = time.time()
				print(now, 'start of ptcloud')
				cloud_list = list(pc2.read_points(val, field_names = ("x", "y", "z"), skip_nans=False))
				now = time.time()
				print(now, 'inter of ptcloud')
				i1=(640*b1)+a1+1
				i2=(640*b2)+a2+1
				i3=(640*b3)+a3+1
				i4=(640*b4)+a4+1

				#now = time.time()
				#print(now, 'start of ptcloud')
				mat1 = cloud_list[i1]
				mat2 = cloud_list[i2]
				mat3 = cloud_list[i3]
				mat4 = cloud_list[i4]
				noww = time.time()
				print(noww, 'end of ptcloud')

				fx=(mat1[0]+mat2[0]+mat3[0]+mat4[0])/4
				fy=(mat1[1]+mat2[1]+mat3[1]+mat4[1])/4
				fz=(mat1[2]+mat2[2]+mat3[2]+mat4[2])/4
				#print(mat4,b4,a4)
				ps = PointStamped()
				ps.header.frame_id = "r200link"
				ps.header.stamp = rospy.Time(0)
				ps.point.x = fx
				ps.point.y = fy
				ps.point.z = fz
				mat = listener.transformPoint("/world", ps)
				#print(mat)

				if (np.abs(mat1[2]-mat2[2])<0.3 and np.abs(mat2[2]-mat3[2])<0.3 and np.abs(mat3[2]-mat4[2])<0.3 and np.abs(mat1[2]-mat4[2])<0.3):
					#print(mat.point)
					if(3.0<mat.point.z<3.5):
						if(3.9<mat.point.x<4.1):
							col[0] = mat.point
						elif(4.9<mat.point.x<5.1):
							col[1] = mat.point
						elif(5.9<mat.point.x<6.1):
							col[2] = mat.point
						elif(6.9<mat.point.x<7.1):
							col[3] = mat.point
						elif(7.9<mat.point.x<8.1):
							col[4] = mat.point
						elif(8.9<mat.point.x<9.1):
							col[5] = mat.point
						elif(9.9<mat.point.x<10.1):
							col[6] = mat.point
						elif(10.9<mat.point.x<11.1):
							col[7] = mat.point
						elif(11.9<mat.point.x<12.1):
							col[8] = mat.point
						elif(12.9<mat.point.x<13.1):
							col[9] = mat.point
						elif(13.9<mat.point.x<14.1):
							col[10] = mat.point
						elif(14.9<mat.point.x<15.1):
							col[11] = mat.point
						elif(15.9<mat.point.x<16.1):
							col[12] = mat.point
						elif(16.9<mat.point.x<17.1):
							col[13] = mat.point
						elif(17.9<mat.point.x<18.1):
							col[14] = mat.point
						print(col,'dfsdjfksbdfhjsbfsdjk')
						frame_list.centers = col

	cv2.imshow('image',img)
	cv2.waitKey(30)
	pub.publish(frame_list)
	#now = rospy.get_rostime()
	#print(now.secs, 'end of callback')
	#rate.sleep()


if __name__ == '__main__':
	rospy.init_node('world_coordinate', anonymous=True)
	print('node_initialised')
	global frame_list
	global col
	frame_list = frame()
	point = Point()
	point.x = 0.0
	point.y = 0.0
	point.z = 0.0
	col = [point]*15
	frame_list.centers = col
	lisn = tf.TransformListener()
	listener = tf.TransformListener()
	rospy.Subscriber("/r200/color/image_raw", Image, cam_frame)
	rospy.Subscriber("/r200/depth/points", PointCloud2, callback)
	pub = rospy.Publisher('frame_center/position', frame, queue_size=100)
	rospy.spin()