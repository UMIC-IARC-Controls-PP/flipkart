#!/usr/bin/env python
# ROS python API
import rospy
import time
import os
import rospy
from frame_detection.msg import frame
from geometry_msgs.msg import PointStamped, PoseStamped, Point


def main():
	rospy.init_node('publisher')
	pubnew = rospy.Publisher('/frame_center/position', frame, queue_size = 10)
	frames = frame()
	wplist = []
	col = [[4.0, 0.0, 3.275],[5.0, -1.0, 3.275], [0.0, 0.0, 0.0], [7.0 ,-1.0 ,3.275], [8.0,0.5 ,3.275], [9, 0.5 ,3.275], [10.0, 1.8, 3.275], [11.0,0.0, 3.275], [12.0, -1.8, 3.275], [13.0 ,0.0 ,3.275], [14.0 ,1.0, 3.275], [15.0, 0.0 ,3.275], [16.0, -1.0 ,3.275], [17.0, 0.5, 3.275], [18.0 ,0.0, 3.275]]
	for wp in col:
		point = Point()
		point.x = wp[0]
		point.y = wp[1]
		point.z = wp[2]
		wplist.append(point)

	print(wplist)
	frames.centers = wplist
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rospy.loginfo(frames)
		pubnew.publish(frames)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass